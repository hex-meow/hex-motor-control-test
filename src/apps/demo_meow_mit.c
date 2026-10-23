/*
 * demo_meow_mit -- drive a single Meow Motor (hexmeow 4310 / 4342) in MIT
 * mode by streaming the UNCOMPRESSED MIT record 0x4102 over RPDO1.
 *
 * Why this demo exists
 * --------------------
 * The uncompressed MIT record is 18 bytes wide, so streaming it needs a
 * CAN-FD frame -- there is no way to squeeze it into classic CAN's 8. That
 * makes it a direct end-to-end test of the FD data path: if the adapter,
 * the bit timing and the motor all agree, the motor moves; if anything in
 * the chain falls back to classic CAN, the frame cannot even be built.
 *
 * The compressed record (0x4103) exists precisely to fit in 8 bytes and is
 * what the hexmeow `demo_mit` uses. This demo deliberately does not use
 * it.
 *
 * Control law (0x4102)
 * --------------------
 *     tau = Tff + clamp( Kp*factor*(Pdes - p) + Kd*factor*(Vdes - v), +-limit )
 *
 * `limit` is 0x4102:06 in permille of peak torque and clamps ONLY the Kp/Kd
 * term; Tff is added outside it. `factor` is the read-only scale from
 * 0x4102:07. This demo leaves Kp and Pdes at zero, so the motor is a pure
 * velocity source with a torque ceiling -- it accelerates towards
 * `vdes_rev_per_s` and never pulls harder than `torque_permille`.
 *
 * Usage:
 *   demo_meow_mit [--verbose|-v] [--slim] [--tff <Nm>] <iface>
 *                 [node_id=1] [vdes_rev_per_s=0.5]
 *                 [kd_nm_per_rev_per_s=0.2] [torque_permille=100]
 *
 *   --slim   stream only 0x4102:02/:03/:05/:06 (12-byte frame) instead of
 *            all six sub-indexes (20-byte frame). Use this if a firmware
 *            build refuses to map :01 or :04 into a PDO.
 *   --tff    constant feed-forward torque in Nm, added OUTSIDE the clamp.
 *            Defaults to 0. It is still bounded by the 0x4572 ceiling.
 *
 * Lifecycle
 * ---------
 *  1. Open the CAN-FD socket and the SDO socket.
 *  2. Run the SDO bring-up (identity, fault clear, PDO mapping, torque
 *     ceiling, MIT mode entry, NMT operational). No host heartbeat is sent
 *     during this phase -- the motor's consumer-heartbeat watchdog only
 *     arms after the first heartbeat, and arming it before a long SDO
 *     sequence would trip fault 0xA4 mid-init.
 *  3. Start the control thread. It emits a heartbeat immediately and then
 *     an all-zero RPDO every 2 ms, which commands zero torque.
 *  4. Wait 100 ms so the motor has seen several heartbeats.
 *  5. Apply the real target. The next RPDO carries it.
 *  6. Print telemetry once a second, including how many received PDOs
 *     actually arrived as FD frames.
 *
 * SIGINT / SIGTERM exit immediately via _exit(0): the motor sees the
 * heartbeat stop within 250 ms and brakes on its own.
 */

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hexmeow_canfd.h"
#include "hexmeow_sdo.h"
#include "meow_common.h"
#include "meow_motor_ctrl.h"
#include "meow_motor_init.h"

/* Heartbeats-only window before the rotation command is applied. */
#define HB_WARMUP_MS  100

static meow_ctrl_t     g_ctrl;
static meow_motor_t    g_motor;
static hexmeow_sdo_t g_sdo;

static void usage(const char *argv0) {
    fprintf(stderr,
        "demo_meow_mit -- Meow Motor uncompressed-MIT over CAN-FD RPDO\n"
        "License: MIT (see LICENSE)\n"
        "\n"
        "Usage: %s [--verbose|-v] [--slim] [--tff <Nm>] <iface>\n"
        "          [node_id=1] [vdes_rev_per_s=0.5] "
        "[kd_nm_per_rev_per_s=0.2] [torque_permille=100]\n"
        "\n"
        "  --slim   stream 0x4102:02/:03/:05/:06 only (12-byte frame)\n"
        "  --tff    constant feed-forward torque in Nm (default 0)\n"
        "\n"
        "The interface must already be up with `fd on`, e.g.\n"
        "  sudo ip link set can0 up type can bitrate 1000000 sample-point 0.8 \\\n"
        "       dbitrate 5000000 dsample-point 0.75 fd on\n",
        argv0);
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
    meow_stream_layout_t layout = MEOW_STREAM_FULL;
    float tff_nm = 0.0f;

    /* Strip flags from argv, leaving only the positional arguments. */
    int out = 0;
    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "--verbose") == 0 || strcmp(argv[i], "-v") == 0) {
            meow_verbose = 1;
        } else if (strcmp(argv[i], "--slim") == 0) {
            layout = MEOW_STREAM_SLIM;
        } else if (strcmp(argv[i], "--tff") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "ERROR: --tff needs a value in Nm\n");
                return 2;
            }
            tff_nm = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            argv[out++] = argv[i];
        }
    }
    argc = out;

    if (argc < 2 || argc > 6) {
        usage(argv[0]);
        return 2;
    }
    const char *ifname     = argv[1];
    int   node_id          = (argc >= 3) ? atoi(argv[2]) : 1;
    float vdes_rev_s       = (argc >= 4) ? (float)atof(argv[3]) : 0.5f;
    float kd_nm_per_rev_s  = (argc >= 5) ? (float)atof(argv[4]) : 0.2f;
    int   torque_permille  = (argc >= 6) ? atoi(argv[5]) : 100;

    if (node_id < 1 || node_id > 127) {
        fprintf(stderr, "ERROR: node_id must be 1..127 (got %d)\n", node_id);
        return 2;
    }
    if (torque_permille < 0 || torque_permille > 1000) {
        fprintf(stderr, "ERROR: torque_permille must be 0..1000 (got %d)\n",
                torque_permille);
        return 2;
    }
    if (kd_nm_per_rev_s < 0.0f) {
        fprintf(stderr, "ERROR: kd must be >= 0 (got %f)\n",
                (double)kd_nm_per_rev_s);
        return 2;
    }

    /* Install the handler first so Ctrl-C works during the SDO init too. */
    install_signal_handlers();

    fprintf(stderr,
            "[demo_meow_mit] iface=%s node=%d vdes=%.3f Rev/s kd=%.4f Nm/(Rev/s) "
            "limit=%d permille tff=%.3f Nm layout=%s\n",
            ifname, node_id, (double)vdes_rev_s, (double)kd_nm_per_rev_s,
            torque_permille, (double)tff_nm,
            (layout == MEOW_STREAM_SLIM) ? "slim" : "full");

    int canfd_fd = hexmeow_canfd_open(ifname);
    hexmeow_sdo_open(&g_sdo, ifname);

    memset(&g_motor, 0, sizeof(g_motor));
    g_motor.node_id = (uint8_t)node_id;
    g_motor.layout = layout;
    /* One knob: the motor-side ceiling (0x4572) matches the MIT clamp, so
     * nothing this demo commands -- clamped term or feed-forward -- can
     * exceed the number the user asked for. */
    g_motor.max_torque_permille = (uint16_t)torque_permille;

    meow_motor_init(&g_motor, &g_sdo, canfd_fd);

    /* Convert the physical gain into the motor's raw domain now that the
     * factory scale factor is known. */
    uint16_t kd_raw = meow_gain_to_raw(kd_nm_per_rev_s, g_motor.kp_kd_factor);
    if (kd_nm_per_rev_s > 0.0f && kd_raw == 0) {
        fprintf(stderr, "[demo_meow_mit] WARN: kd=%.4f Nm/(Rev/s) rounds to 0 "
                        "raw units (factor=%.6f); the motor will not move\n",
                (double)kd_nm_per_rev_s, (double)g_motor.kp_kd_factor);
    }
    fprintf(stderr, "[demo_meow_mit] kd %.4f Nm/(Rev/s) -> raw %u; "
                    "torque clamp %d permille of %.3f Nm = %.3f Nm\n",
            (double)kd_nm_per_rev_s, (unsigned)kd_raw, torque_permille,
            (double)g_motor.peak_torque_nm,
            (double)g_motor.peak_torque_nm * torque_permille / 1000.0);

    meow_ctrl_init(&g_ctrl, &g_motor, canfd_fd);
    meow_ctrl_start(&g_ctrl);

    fprintf(stderr, "[demo_meow_mit] heartbeat warm-up (%d ms) before "
                    "applying the target\n", HB_WARMUP_MS);
    usleep(HB_WARMUP_MS * 1000);

    meow_mit_target_t target = {
        .pdes_rev       = 0.0f,   /* unused: Kp is zero */
        .vdes_rev_s     = vdes_rev_s,
        .tff_nm         = tff_nm,
        .kp_raw         = 0,      /* pure velocity tracking */
        .kd_raw         = kd_raw,
        .limit_permille = (uint16_t)torque_permille,
    };
    meow_ctrl_set_target(&g_ctrl, &target);

    if (meow_verbose) {
        uint8_t frame[MEOW_RPDO_LEN_FULL];
        uint8_t len = meow_rpdo_len(layout);
        meow_mit_target_pack(&target, layout, frame);
        fprintf(stderr, "[v] RPDO1 cob=0x%03X len=%u payload:",
                (unsigned)(MEOW_COB_RPDO1 | (unsigned)node_id), (unsigned)len);
        for (uint8_t i = 0; i < len; i++) fprintf(stderr, " %02X", frame[i]);
        fputc('\n', stderr);
    }

    uint32_t last_rx_fd = 0, last_rx_classic = 0, last_rpdo = 0;
    for (;;) {
        sleep(1);

        meow_motor_state_t s;
        uint32_t rpdo_count = 0;
        meow_ctrl_snapshot(&g_ctrl, &s, &rpdo_count);

        float torque_nm = (float)s.torque_permille * g_motor.peak_torque_nm / 1000.0f;
        fprintf(stderr,
                "[demo_meow_mit] pos=%+9.4f rev  torque=%+5d permille (%+.3f Nm)  "
                "mode=0x%02X (%s)  err=0x%04X  drv=%.1fC mtr=%.1fC\n",
                (double)s.position_rev, (int)s.torque_permille,
                (double)torque_nm, s.mode_display,
                meow_mode_display_str(s.mode_display), s.error_code,
                (double)s.driver_temp_x10 * 0.1, (double)s.motor_temp_x10 * 0.1);

        fprintf(stderr,
                "[demo_meow_mit]   CAN-FD  tx %u RPDO/s (%u B)   "
                "rx %u FD/s + %u classic/s   (tpdo1=%u tpdo2=%u total)\n",
                (unsigned)(rpdo_count - last_rpdo),
                (unsigned)meow_rpdo_len(layout),
                (unsigned)(s.rx_fd - last_rx_fd),
                (unsigned)(s.rx_classic - last_rx_classic),
                (unsigned)s.tpdo1_count, (unsigned)s.tpdo2_count);

        if (s.rx_classic > last_rx_classic) {
            /* Not a failure by itself: with 0x2100:03 (BRS) set to 0 the
             * motor sends its <=8-byte TPDOs as classic CAN on purpose, to
             * stay compatible with non-FD networks. */
            fprintf(stderr, "[demo_meow_mit]   note: some TPDOs arrived as "
                            "classic CAN -- check 0x2100:03 (BRS enable) on "
                            "the motor\n");
        }
        if (s.tpdo1_count == 0) {
            fprintf(stderr, "[demo_meow_mit]   WARN: no TPDO1 received yet -- "
                            "the motor is not talking back\n");
        }
        if (MEOW_DISPLAY_IS_ERROR(s.mode_display)) {
            fprintf(stderr, "[demo_meow_mit]   FAULT 0x%02X (%s), detail "
                            "0x%04X -- the motor has stopped\n",
                    s.mode_display, meow_mode_display_str(s.mode_display),
                    s.error_code);
        }

        last_rpdo = rpdo_count;
        last_rx_fd = s.rx_fd;
        last_rx_classic = s.rx_classic;
    }
    return 0;  /* unreachable */
}
