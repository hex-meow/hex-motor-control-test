/*
 * demo_velocity -- spin every connected hexfellow motor at a constant
 * velocity using profile-velocity mode (mode 3) with a 6072h max-torque
 * limit.
 *
 * Usage:
 *   demo_velocity <iface> <motor_count>
 *                 [target_rev_per_s] [torque_permille]
 *
 * Defaults: target_rev_per_s = 0.5, torque_permille = 200
 *
 * Layout for one-to-many control: each motor takes 6 bytes in the
 * shared 0x190 RPDO:
 *   [u16 6072h max torque][f32 60FFh target velocity]
 *
 * Up to 8 motors -> 48 bytes (rounded to a 48-byte CAN-FD payload).
 *
 * See `demo_mit.c` for the lifecycle commentary -- this demo follows the
 * same heartbeat-before-rotation contract.
 */

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hexfellow_canfd.h"
#include "hexfellow_common.h"
#include "hexfellow_motor_ctrl.h"
#include "hexfellow_motor_init.h"
#include "hexfellow_sdo.h"

#define HB_WARMUP_MS  100

static hexfellow_ctrl_t       g_ctrl;
static hexfellow_motor_set_t  g_set;
static hexfellow_sdo_t        g_sdo;
static int                    g_canfd_fd = -1;

static void usage(const char *argv0) {
    fprintf(stderr,
        "Usage: %s [--verbose|-v] <iface> <motor_count> "
        "[target_rev_per_s=0.5] [torque_permille=200]\n",
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
    if (argc < 3 || argc > 5) {
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
    int   torque_pm    = (argc >= 5) ? atoi(argv[4]) : 200;
    if (torque_pm < 0 || torque_pm > 1000) {
        fprintf(stderr, "ERROR: torque_permille must be 0..1000 (got %d)\n",
                torque_pm);
        return 2;
    }

    install_signal_handlers();

    fprintf(stderr,
            "[demo_velocity] iface=%s count=%d target=%.3f Rev/s torque=%d permille\n",
            ifname, count, target_rev_s, torque_pm);

    g_canfd_fd = hexfellow_canfd_open(ifname);
    hexfellow_sdo_open(&g_sdo, ifname);

    memset(&g_set, 0, sizeof(g_set));
    g_set.mode = HEXFELLOW_MODE_KIND_VELOCITY;
    g_set.count = (uint8_t)count;
    g_set.torque_permille = (uint16_t)torque_pm;
    g_set.kp_kd_torque_permille = 0;            /* unused */
    g_set.mapping = hexfellow_mit_mapping_default(); /* unused */

    hexfellow_motor_set_init(&g_set, &g_sdo, g_canfd_fd);

    for (uint8_t i = 0; i < g_set.count; i++) {
        fprintf(stderr,
                "[demo_velocity] motor %u: id=%u fw=%u serial=0x%08X peak=%.0f mNm\n",
                (unsigned)i, (unsigned)g_set.motors[i].node_id,
                (unsigned)g_set.motors[i].firmware_version,
                (unsigned)g_set.motors[i].serial_number,
                g_set.motors[i].peak_torque_mNm);
    }

    /* Targets default to zero (no torque, no velocity) -- the motor will
     * not spin until we update them after the heartbeat warm-up. */
    hexfellow_ctrl_init(&g_ctrl, &g_set, g_canfd_fd);
    hexfellow_ctrl_start(&g_ctrl);

    fprintf(stderr,
            "[demo_velocity] heartbeat warm-up (%d ms) before applying targets\n",
            HB_WARMUP_MS);
    usleep(HB_WARMUP_MS * 1000);

    for (uint8_t i = 0; i < g_set.count; i++) {
        hexfellow_ctrl_set_velocity_target(&g_ctrl, i, target_rev_s,
                                           (uint16_t)torque_pm);
    }

    for (;;) {
        sleep(1);
        for (uint8_t i = 0; i < g_set.count; i++) {
            hexfellow_motor_state_t s;
            hexfellow_ctrl_snapshot(&g_ctrl, i, &s);
            float peak = g_set.motors[i].peak_torque_mNm * 0.001f;
            float torque_nm = (float)s.raw_torque_permille * peak * 0.001f;
            float drv_c = (float)s.driver_temp_x10 * 0.1f;
            float mtr_c = (float)s.motor_temp_x10 * 0.1f;
            fprintf(stderr,
                    "[demo_velocity] m%u pos=%+.4f rev (turns=%d) torque=%+.3f Nm "
                    "status=0x%04X err=0x%04X drv=%.1fC mtr=%.1fC\n",
                    (unsigned)i, s.position_rev, (int)s.multi_turns, torque_nm,
                    s.status_word, s.error_code, drv_c, mtr_c);
        }
    }
    return 0;  /* unreachable */
}
