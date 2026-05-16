/*
 * demo_mit -- spin every connected hexfellow motor at a constant velocity
 * using MIT mode (mode 5).
 *
 * Usage:
 *   demo_mit <iface> <motor_count> [target_rev_per_s] [kd] [torque_permille]
 *
 * Defaults: target_rev_per_s = 0.5, kd = 2.0, torque_permille = 800.
 *
 * Lifecycle
 * ---------
 *  1. Open the CAN-FD socket and the SDO socket.
 *  2. Run the per-motor SDO initialization (factory-UID / version checks,
 *     PDO mapping, consumer-heartbeat configuration, CiA402 enable,
 *     NMT operational). NO master heartbeat is sent during this phase.
 *  3. Start the control thread. It immediately begins emitting master
 *     heartbeats on cob 0x710 and RPDO frames on cob 0x190; the RPDO
 *     payload is all zeros until the application updates the targets, so
 *     the motors stay still.
 *  4. Sleep ~100 ms so the motors receive several heartbeats BEFORE we
 *     send any rotation command. Per the firmware contract, the motor's
 *     consumer-heartbeat watchdog (1016h-01, 250 ms) only arms after it
 *     has seen the first master heartbeat -- so as long as heartbeats are
 *     flowing before we start commanding motion, the motor will not trip
 *     0x8130 "Heartbeat error".
 *  5. Set the MIT target with the requested velocity. The next RPDO frame
 *     (emitted within 1 ms) will include the new target.
 *  6. Loop forever printing per-second telemetry. SIGINT / SIGTERM end the
 *     process immediately via `_exit(0)` -- the motor detects heartbeat
 *     loss on its own and stops spinning, so no graceful shutdown is
 *     required.
 *
 * Hard constraints (see README.md):
 *   - CANopen IDs must start at 1 and be sequential.
 *   - CAN interface must be UP with `fd on` before running.
 *   - Firmware version must be exactly 8 (see 1018h-03).
 */

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hexfellow_canfd.h"
#include "hexfellow_common.h"
#include "hexfellow_mit_target.h"
#include "hexfellow_motor_ctrl.h"
#include "hexfellow_motor_init.h"
#include "hexfellow_sdo.h"

/* Time the control thread spends emitting heartbeats with a zero target
 * before the rotation command is applied. 100 ms = at least the first
 * heartbeat plus a buffer; well under the motor's 250 ms timeout. */
#define HB_WARMUP_MS  100

static hexfellow_ctrl_t       g_ctrl;
static hexfellow_motor_set_t  g_set;
static hexfellow_sdo_t        g_sdo;
static int                    g_canfd_fd = -1;

static void usage(const char *argv0) {
    fprintf(stderr,
        "demo_mit -- hexfellow CANopen MIT-mode motor demo\n"
        "Author: kisonhe <kisonhe@hexfellow.com>\n"
        "License: MIT (see LICENSE)\n"
        "\n"
        "Usage: %s [--verbose|-v] <iface> <motor_count> "
        "[target_rev_per_s=0.5] [kd=2.0] [torque_permille=800]\n",
        argv0);
}

/* Strip "--verbose" / "-v" from argv (any position), setting the global
 * `hexfellow_verbose` to 1 if present. Returns the new argc. */
static int strip_verbose_flag(int argc, char *argv[]) {
    int out = 0;
    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "--verbose") == 0 || strcmp(argv[i], "-v") == 0) {
            hexfellow_verbose = 1;
        } else {
            argv[out++] = argv[i];
        }
    }
    return out;
}

/* SIGINT / SIGTERM handler. There is intentionally no graceful shutdown:
 * the motors will trip their own consumer-heartbeat watchdogs (250 ms) and
 * stop on their own. `_exit` is async-signal-safe; `exit` is not. */
static void on_sigint(int sig) {
    (void)sig;
    _exit(0);
}

static void install_signal_handlers(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_sigint;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

int main(int argc, char *argv[]) {
    argc = strip_verbose_flag(argc, argv);
    if (argc < 3 || argc > 6) {
        usage(argv[0]);
        return 2;
    }
    const char *ifname = argv[1];
    int count = atoi(argv[2]);
    if (count <= 0 || count > HEXFELLOW_MAX_MOTORS) {
        fprintf(stderr, "ERROR: motor_count must be 1..%d (got %d)\n",
                HEXFELLOW_MAX_MOTORS, count);
        return 2;
    }
    float target_rev_s = (argc >= 4) ? (float)atof(argv[3]) : 0.5f;
    float kd           = (argc >= 5) ? (float)atof(argv[4]) : 2.0f;
    int   torque_pm    = (argc >= 6) ? atoi(argv[5]) : (int)HEXFELLOW_DEFAULT_TORQUE_PERMILLE;
    if (torque_pm < 0 || torque_pm > 1000) {
        fprintf(stderr, "ERROR: torque_permille must be 0..1000 (got %d)\n",
                torque_pm);
        return 2;
    }

    /* Install the signal handler first so Ctrl-C terminates the program
     * at any point -- including during the (potentially slow) SDO init. */
    install_signal_handlers();

    fprintf(stderr,
            "[demo_mit] iface=%s count=%d target=%.3f Rev/s kd=%.3f torque=%d permille\n",
            ifname, count, target_rev_s, kd, torque_pm);

    g_canfd_fd = hexfellow_canfd_open(ifname);
    hexfellow_sdo_open(&g_sdo, ifname);

    memset(&g_set, 0, sizeof(g_set));
    g_set.mode = HEXFELLOW_MODE_KIND_MIT;
    g_set.count = (uint8_t)count;
    g_set.torque_permille = (uint16_t)torque_pm;
    /* Slightly conservative MIT KP/KD output torque limit (~77%). */
    g_set.kp_kd_torque_permille = 770;
    g_set.mapping = hexfellow_mit_mapping_default();

    /* Full SDO init: UID/version checks, PDO remap, CiA402 enable, NMT
     * operational. No heartbeats are emitted while this runs. */
    hexfellow_motor_set_init(&g_set, &g_sdo, g_canfd_fd);

    for (uint8_t i = 0; i < g_set.count; i++) {
        fprintf(stderr,
                "[demo_mit] motor %u: id=%u fw=%u serial=0x%08X peak=%.0f mNm\n",
                (unsigned)i, (unsigned)g_set.motors[i].node_id,
                (unsigned)g_set.motors[i].firmware_version,
                (unsigned)g_set.motors[i].serial_number,
                g_set.motors[i].peak_torque_mNm);
    }

    /* Targets default to zero -- no motion until we update them below. */
    hexfellow_ctrl_init(&g_ctrl, &g_set, g_canfd_fd);

    /* Start the control thread. Heartbeats start flowing immediately;
     * RPDOs carry a zero target until step 6 of the lifecycle. */
    hexfellow_ctrl_start(&g_ctrl);

    /* Let a few heartbeats reach every motor before commanding rotation. */
    fprintf(stderr,
            "[demo_mit] heartbeat warm-up (%d ms) before applying targets\n",
            HB_WARMUP_MS);
    usleep(HB_WARMUP_MS * 1000);

    /* Apply the actual MIT target. The next RPDO (within 1 ms) will carry
     * this. */
    hexfellow_mit_target_t target = {
        .position = 0.0f,
        .velocity = target_rev_s,
        .torque   = 0.0f,
        .kp       = 0.0f,    /* pure velocity tracking */
        .kd       = kd,
    };
    for (uint8_t i = 0; i < g_set.count; i++) {
        hexfellow_ctrl_set_mit_target(&g_ctrl, i, &target);
    }

    /* Run forever printing telemetry. SIGINT -> _exit(0). */
    for (;;) {
        sleep(1);
        for (uint8_t i = 0; i < g_set.count; i++) {
            hexfellow_motor_state_t s;
            hexfellow_ctrl_snapshot(&g_ctrl, i, &s);
            float peak = g_set.motors[i].peak_torque_mNm * 0.001f; /* Nm */
            float torque_nm = (float)s.raw_torque_permille * peak * 0.001f;
            float drv_c = (float)s.driver_temp_x10 * 0.1f;
            float mtr_c = (float)s.motor_temp_x10 * 0.1f;
            fprintf(stderr,
                    "[demo_mit] m%u pos=%+.4f rev (turns=%d) torque=%+.3f Nm "
                    "status=0x%04X err=0x%04X drv=%.1fC mtr=%.1fC\n",
                    (unsigned)i, s.position_rev, (int)s.multi_turns, torque_nm,
                    s.status_word, s.error_code, drv_c, mtr_c);
        }
    }
    return 0;  /* unreachable */
}
