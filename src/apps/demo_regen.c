/*
 * demo_regen -- drive every connected hexfellow motor through a
 * synchronized full-torque accelerate / cruise / brake-and-reverse
 * cycle. Built for bench-testing regenerative-braking absorption
 * hardware (back-EMF clamp / brake-chopper boards): during every
 * deceleration ramp all motors pump energy back into the DC bus at the
 * same time, which is the worst case the absorber has to survive.
 *
 * Commanded velocity waveform (all motors identical, MIT mode):
 *
 *   +peak ___________                          ___________
 *        /|          |\                       /|
 *       / |          | \                     / |
 *      /  |          |  \                   /  |
 *   0 /   |          |   \        |        /   |     ...
 *         |          |    \       |       /
 *         |          |     \      |      /
 *   -peak |          |      \_____|_____/
 *     ramp   cruise    brake   cruise  brake ...
 *
 * Ramps where |velocity| grows use `accel_rev_s2`; ramps where
 * |velocity| shrinks (braking -- the regen part) use `decel_rev_s2`.
 * That lets you spin up gently but brake violently, keeping the supply
 * current low while maximizing the regen spike.
 *
 * Usage:
 *   demo_regen <iface> <motor_count>
 *              [peak_rev_s] [accel_rev_s2] [decel_rev_s2]
 *              [cruise_s] [torque_permille] [kd]
 *
 * Defaults: peak_rev_s = 1.0, accel_rev_s2 = 2.0, decel_rev_s2 = accel,
 *           cruise_s = 1.0, torque_permille = 800, kd = 2.0.
 *
 * Notes for tuning the test:
 *   - "Full force" is bounded by `torque_permille` (6072h) and by the
 *     MIT KP/KD output limit (2004h-0E) -- this demo sets both to the
 *     same CLI value, unlike demo_mit's conservative hard-coded 770.
 *   - A very steep `decel_rev_s2` (say 100) turns the brake ramp into
 *     an effective full-torque stop: the commanded slope exceeds what
 *     the motor can follow and the torque limit becomes the knob.
 *   - `cruise_s = 0` degenerates into a triangle wave (no cruise).
 *   - `kd` is the MIT velocity-tracking gain; higher kd = harder
 *     tracking = more braking torque for the same slope.
 *   - Phase transitions are printed with a ">>>" prefix and a
 *     monotonic timestamp so you can line them up with scope captures.
 *
 * The motors reverse direction every cycle -- make sure whatever is
 * bolted to them is happy with that.
 *
 * See `demo_mit.c` for the lifecycle commentary -- this demo follows
 * the same heartbeat-before-rotation contract.
 */

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "hexfellow_canfd.h"
#include "hexfellow_common.h"
#include "hexfellow_mit_target.h"
#include "hexfellow_motor_ctrl.h"
#include "hexfellow_motor_init.h"
#include "hexfellow_sdo.h"

#define HB_WARMUP_MS  100

/* Waveform generator tick. Matches the RPDO cadence -- updating the
 * target faster than it is shipped to the bus buys nothing. */
#define WAVE_TICK_US  (HEXFELLOW_RPDO_PERIOD_MS * 1000)

/* Telemetry print period (seconds). */
#define PRINT_PERIOD_S  1.0

static hexfellow_ctrl_t       g_ctrl;
static hexfellow_motor_set_t  g_set;
static hexfellow_sdo_t        g_sdo;
static int                    g_canfd_fd = -1;

static void usage(const char *argv0) {
    fprintf(stderr,
        "demo_regen -- hexfellow synchronized accel/brake cycle "
        "(regen / back-EMF absorber test)\n"
        "Author: kisonhe <kisonhe@hexfellow.com>\n"
        "License: MIT (see LICENSE)\n"
        "\n"
        "Usage: %s [--verbose|-v] <iface> <motor_count>\n"
        "          [peak_rev_s=1.0] [accel_rev_s2=2.0] [decel_rev_s2=accel]\n"
        "          [cruise_s=1.0] [torque_permille=800] [kd=2.0]\n"
        "\n"
        "  peak_rev_s       cruise speed; cycle runs +peak -> -peak -> +peak\n"
        "  accel_rev_s2     slope while |velocity| grows (motoring)\n"
        "  decel_rev_s2     slope while |velocity| shrinks (braking / regen)\n"
        "  cruise_s         time held at +/-peak between ramps (0 = triangle)\n"
        "  torque_permille  6072h AND 2004h-0E limit -- the 'full force' cap\n"
        "  kd               MIT velocity-tracking gain\n",
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

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

/* Move `cmd` one tick toward `tgt`. The slope is `accel` while the step
 * takes |cmd| away from zero (motoring) and `decel` while it takes |cmd|
 * toward zero (braking). A step that crosses zero uses the braking rate
 * for the whole step -- at a 2 ms tick the error is negligible. */
static float slew_toward(float cmd, float tgt, float accel, float decel,
                         float dt) {
    if (cmd == tgt) return cmd;
    float dir = (tgt > cmd) ? 1.0f : -1.0f;
    float rate = (cmd * dir >= 0.0f) ? accel : decel;
    float next = cmd + dir * rate * dt;
    if ((dir > 0.0f && next > tgt) || (dir < 0.0f && next < tgt)) next = tgt;
    return next;
}

int main(int argc, char *argv[]) {
    argc = strip_verbose_flag(argc, argv);
    if (argc < 3 || argc > 9) {
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
    float peak      = (argc >= 4) ? (float)atof(argv[3]) : 1.0f;
    float accel     = (argc >= 5) ? (float)atof(argv[4]) : 2.0f;
    float decel     = (argc >= 6) ? (float)atof(argv[5]) : accel;
    float cruise_s  = (argc >= 7) ? (float)atof(argv[6]) : 1.0f;
    int   torque_pm = (argc >= 8) ? atoi(argv[7]) : (int)HEXFELLOW_DEFAULT_TORQUE_PERMILLE;
    float kd        = (argc >= 9) ? (float)atof(argv[8]) : 2.0f;

    hexfellow_mit_mapping_t mapping = hexfellow_mit_mapping_default();
    if (peak <= 0.0f || peak > mapping.velocity_max) {
        fprintf(stderr, "ERROR: peak_rev_s must be 0..%.1f (got %.3f); "
                "the MIT mapping clamps velocity beyond that\n",
                mapping.velocity_max, peak);
        return 2;
    }
    if (accel <= 0.0f || decel <= 0.0f) {
        fprintf(stderr, "ERROR: accel/decel must be > 0 "
                "(got %.3f / %.3f)\n", accel, decel);
        return 2;
    }
    if (cruise_s < 0.0f) {
        fprintf(stderr, "ERROR: cruise_s must be >= 0 (got %.3f)\n", cruise_s);
        return 2;
    }
    if (torque_pm < 0 || torque_pm > 1000) {
        fprintf(stderr, "ERROR: torque_permille must be 0..1000 (got %d)\n",
                torque_pm);
        return 2;
    }
    if (kd < 0.0f || kd > mapping.kd_max) {
        fprintf(stderr, "ERROR: kd must be 0..%.1f (got %.3f); "
                "the MIT mapping clamps kd beyond that\n",
                mapping.kd_max, kd);
        return 2;
    }

    install_signal_handlers();

    fprintf(stderr,
            "[demo_regen] iface=%s count=%d peak=%.3f Rev/s accel=%.3f "
            "decel=%.3f Rev/s^2 cruise=%.3f s torque=%d permille kd=%.3f\n",
            ifname, count, peak, accel, decel, cruise_s, torque_pm, kd);

    g_canfd_fd = hexfellow_canfd_open(ifname);
    hexfellow_sdo_open(&g_sdo, ifname);

    memset(&g_set, 0, sizeof(g_set));
    g_set.mode = HEXFELLOW_MODE_KIND_MIT;
    g_set.count = (uint8_t)count;
    g_set.torque_permille = (uint16_t)torque_pm;
    /* Unlike demo_mit (hard-coded 770), give the MIT KP/KD path the full
     * requested torque authority -- braking at the real limit is the
     * whole point of this test. */
    g_set.kp_kd_torque_permille = (uint16_t)torque_pm;
    g_set.mapping = mapping;

    hexfellow_motor_set_init(&g_set, &g_sdo, g_canfd_fd);

    for (uint8_t i = 0; i < g_set.count; i++) {
        fprintf(stderr,
                "[demo_regen] motor %u: id=%u fw=%u serial=0x%08X peak=%.0f mNm\n",
                (unsigned)i, (unsigned)g_set.motors[i].node_id,
                (unsigned)g_set.motors[i].firmware_version,
                (unsigned)g_set.motors[i].serial_number,
                g_set.motors[i].peak_torque_mNm);
    }

    /* Targets default to zero -- no motion until the waveform loop below
     * starts pushing commands after the heartbeat warm-up. */
    hexfellow_ctrl_init(&g_ctrl, &g_set, g_canfd_fd);
    hexfellow_ctrl_start(&g_ctrl);

    fprintf(stderr,
            "[demo_regen] heartbeat warm-up (%d ms) before applying targets\n",
            HB_WARMUP_MS);
    usleep(HB_WARMUP_MS * 1000);

    /* Baselines for the measured-velocity estimate (position delta over
     * the telemetry interval). */
    double prev_total[HEXFELLOW_MAX_MOTORS];
    for (uint8_t i = 0; i < g_set.count; i++) {
        hexfellow_motor_state_t s;
        hexfellow_ctrl_snapshot(&g_ctrl, i, &s);
        prev_total[i] = (double)s.multi_turns + (double)s.position_rev;
    }

    hexfellow_mit_target_t target = {
        .position = 0.0f,
        .velocity = 0.0f,
        .torque   = 0.0f,
        .kp       = 0.0f,    /* pure velocity tracking */
        .kd       = kd,
    };

    enum { PHASE_RAMP, PHASE_CRUISE } phase = PHASE_RAMP;
    float  plateau = peak;      /* destination of the current ramp */
    float  cmd = 0.0f;          /* commanded velocity, slewed each tick */
    const double t0 = now_s();
    double prev_t = t0;
    double last = t0;
    double cruise_end = 0.0;
    double next_print = t0 + PRINT_PERIOD_S;

    fprintf(stderr, "[demo_regen] >>> t=%.3f ramp %+.2f -> %+.2f Rev/s\n",
            0.0, cmd, plateau);

    for (;;) {
        usleep(WAVE_TICK_US);
        double t = now_s();
        float dt = (float)(t - last);
        last = t;
        /* Guard against a huge step after SIGSTOP / system suspend. */
        if (dt > 0.1f) dt = 0.1f;

        if (phase == PHASE_RAMP) {
            cmd = slew_toward(cmd, plateau, accel, decel, dt);
            if (cmd == plateau) {
                phase = PHASE_CRUISE;
                cruise_end = t + (double)cruise_s;
                fprintf(stderr,
                        "[demo_regen] >>> t=%.3f cruise at %+.2f Rev/s for %.2f s\n",
                        t - t0, cmd, cruise_s);
            }
        } else if (t >= cruise_end) {
            plateau = -plateau;
            phase = PHASE_RAMP;
            fprintf(stderr,
                    "[demo_regen] >>> t=%.3f ramp %+.2f -> %+.2f Rev/s "
                    "(brake @ %.2f, accel @ %.2f Rev/s^2)\n",
                    t - t0, cmd, plateau, decel, accel);
        }

        /* Same command to every motor -- the whole point is that the
         * regen currents of all motors add up on the shared bus. */
        target.velocity = cmd;
        for (uint8_t i = 0; i < g_set.count; i++) {
            hexfellow_ctrl_set_mit_target(&g_ctrl, i, &target);
        }

        if (t >= next_print) {
            double span = t - prev_t;
            for (uint8_t i = 0; i < g_set.count; i++) {
                hexfellow_motor_state_t s;
                hexfellow_ctrl_snapshot(&g_ctrl, i, &s);
                double total = (double)s.multi_turns + (double)s.position_rev;
                float meas = (span > 0.0) ? (float)((total - prev_total[i]) / span)
                                          : 0.0f;
                prev_total[i] = total;
                float pk = g_set.motors[i].peak_torque_mNm * 0.001f; /* Nm */
                float torque_nm = (float)s.raw_torque_permille * pk * 0.001f;
                float drv_c = (float)s.driver_temp_x10 * 0.1f;
                float mtr_c = (float)s.motor_temp_x10 * 0.1f;
                fprintf(stderr,
                        "[demo_regen] t=%.1f m%u cmd=%+.2f meas=%+.2f Rev/s "
                        "torque=%+.3f Nm status=0x%04X err=0x%04X "
                        "drv=%.1fC mtr=%.1fC\n",
                        t - t0, (unsigned)i, cmd, meas, torque_nm,
                        s.status_word, s.error_code, drv_c, mtr_c);
            }
            prev_t = t;
            next_print += PRINT_PERIOD_S;
        }
    }
    return 0;  /* unreachable */
}
