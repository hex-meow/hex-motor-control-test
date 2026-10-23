#include "hexmeow_motor_ctrl.h"
#include "hexmeow_canfd.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* Global verbose flag (declared in hexmeow_common.h). Set by the demo
 * mains when the user passes --verbose / -v. */
int hexmeow_verbose = 0;

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)(ts.tv_nsec / 1000000);
}

static void die_io(const char *what) {
    fprintf(stderr, "[hexmeow_ctrl] FATAL: %s: %s\n", what, strerror(errno));
    exit(EXIT_FAILURE);
}

void hexmeow_ctrl_init(hexmeow_ctrl_t *c, hexmeow_motor_set_t *set, int canfd_fd) {
    memset(c, 0, sizeof(*c));
    c->canfd_fd = canfd_fd;
    c->set = set;
    pthread_mutex_init(&c->lock, NULL);
    /* Velocity defaults: zero target, zero torque cap -> no torque output.
     * The MIT target struct is already zeroed by memset, which means kp=kd=0
     * and position/velocity/torque = 0 -- no rotation will be commanded. The
     * caller is expected to update targets via `_set_mit_target` or
     * `_set_velocity_target` AFTER heartbeats have been flowing for a short
     * warm-up period. */
    for (uint8_t i = 0; i < HEXMEOW_MAX_MOTORS; i++) {
        c->vel_target_rev_s[i] = 0.0f;
        c->vel_torque_permille[i] = 0;
    }
}

void hexmeow_ctrl_set_mit_target(hexmeow_ctrl_t *c, uint8_t index,
                                   const hexmeow_mit_target_t *t) {
    pthread_mutex_lock(&c->lock);
    c->mit_targets[index] = *t;
    pthread_mutex_unlock(&c->lock);
}

void hexmeow_ctrl_set_velocity_target(hexmeow_ctrl_t *c, uint8_t index,
                                        float target_rev_s,
                                        uint16_t torque_permille) {
    pthread_mutex_lock(&c->lock);
    c->vel_target_rev_s[index] = target_rev_s;
    c->vel_torque_permille[index] = torque_permille;
    pthread_mutex_unlock(&c->lock);
}

void hexmeow_ctrl_snapshot(hexmeow_ctrl_t *c, uint8_t index,
                             hexmeow_motor_state_t *out) {
    pthread_mutex_lock(&c->lock);
    *out = c->state[index];
    pthread_mutex_unlock(&c->lock);
}

/* --- TPDO parsing ------------------------------------------------------- */

/* Single-turn -> multi-turn delta tracking. The motor reports a single-turn
 * position in [0, 1) Rev; we accumulate full-turn rollovers here. */
static int32_t update_multi_turn(int32_t prev_turns, float prev_pos,
                                 float new_pos) {
    float diff = new_pos - prev_pos;
    if (diff > 0.5f)  return prev_turns - 1;
    if (diff < -0.5f) return prev_turns + 1;
    return prev_turns;
}

static void handle_tpdo1(hexmeow_ctrl_t *c, uint8_t index,
                         const uint8_t *data) {
    float pos;
    memcpy(&pos, data + 0, 4);
    uint32_t ts = (uint32_t)data[4]
                | ((uint32_t)data[5] << 8)
                | ((uint32_t)data[6] << 16)
                | ((uint32_t)data[7] << 24);
    int16_t torque_pm = (int16_t)((uint16_t)data[8] | ((uint16_t)data[9] << 8));
    uint16_t err = (uint16_t)data[10] | ((uint16_t)data[11] << 8);

    pthread_mutex_lock(&c->lock);
    hexmeow_motor_state_t *s = &c->state[index];
    s->multi_turns = update_multi_turn(s->multi_turns, s->position_rev, pos);
    s->position_rev = pos;
    s->timestamp_us = ts;
    s->raw_torque_permille = torque_pm;
    s->error_code = err;
    s->last_tpdo1_ms = now_ms();
    pthread_mutex_unlock(&c->lock);
}

static void handle_tpdo2(hexmeow_ctrl_t *c, uint8_t index,
                         const uint8_t *data) {
    uint16_t status = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    int16_t  drv_t  = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
    int16_t  mtr_t  = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
    uint16_t ctrl   = (uint16_t)data[6] | ((uint16_t)data[7] << 8);

    pthread_mutex_lock(&c->lock);
    hexmeow_motor_state_t *s = &c->state[index];
    s->status_word = status;
    s->driver_temp_x10 = drv_t;
    s->motor_temp_x10 = mtr_t;
    s->control_word = ctrl;
    s->last_tpdo2_ms = now_ms();
    pthread_mutex_unlock(&c->lock);
}

/* Drain everything that's currently in the kernel RX queue. recv() is
 * MSG_DONTWAIT so this returns the instant the queue is empty -- no
 * waiting, no chance of being held up by the next incoming TPDO. */
static void drain_rx(hexmeow_ctrl_t *c) {
    int n_t1 = 0, n_t2 = 0, n_other = 0;
    for (;;) {
        struct canfd_frame f;
        int is_fd = 0;
        int rc = hexmeow_canfd_recv(c->canfd_fd, &f, &is_fd);
        if (rc == -2) break;          /* queue empty */
        if (rc == -1) {
            fprintf(stderr, "[hexmeow_ctrl] WARN: recv failed: %s\n",
                    strerror(errno));
            break;
        }
        uint16_t cob = (uint16_t)(f.can_id & 0x7FFu);
        uint8_t  node = (uint8_t)(cob & 0x7Fu);
        if (node == 0 || node > c->set->count) { n_other++; continue; }
        uint16_t base = (uint16_t)(cob & 0x780u);
        uint8_t index = (uint8_t)(node - 1);
        if (base == HEXMEOW_COB_TPDO1 && f.len >= HEXMEOW_TPDO1_LEN) {
            handle_tpdo1(c, index, f.data);
            n_t1++;
        } else if (base == HEXMEOW_COB_TPDO2 && f.len >= HEXMEOW_TPDO2_LEN) {
            handle_tpdo2(c, index, f.data);
            n_t2++;
        } else {
            n_other++;
        }
    }
    if (hexmeow_verbose && (n_t1 | n_t2 | n_other)) {
        fprintf(stderr, "[v] %llums drain: T1=%d T2=%d other=%d\n",
                (unsigned long long)now_ms(), n_t1, n_t2, n_other);
    }
}

/* --- TX --------------------------------------------------------------- */

static void send_heartbeat(hexmeow_ctrl_t *c) {
    uint8_t hb = 0x05;  /* Operational state */
    uint16_t cob = HEXMEOW_COB_HB | HEXMEOW_MASTER_NODE_ID;
    if (hexmeow_canfd_send_classic(c->canfd_fd, cob, &hb, 1) != 0) {
        die_io("send heartbeat");
    }
    if (hexmeow_verbose) {
        fprintf(stderr, "[v] %llums HB sent (0x%03X)\n",
                (unsigned long long)now_ms(), cob);
    }
}

static void build_and_send_rpdo_mit(hexmeow_ctrl_t *c) {
    uint8_t  data[HEXMEOW_MAX_MOTORS * HEXMEOW_MIT_BYTES_PER_MOTOR] = {0};
    uint8_t  cnt = c->set->count;

    pthread_mutex_lock(&c->lock);
    for (uint8_t i = 0; i < cnt; i++) {
        hexmeow_mit_target_pack(&c->mit_targets[i], &c->set->mapping,
                                  &data[i * HEXMEOW_MIT_BYTES_PER_MOTOR]);
    }
    pthread_mutex_unlock(&c->lock);

    uint8_t total = (uint8_t)(cnt * HEXMEOW_MIT_BYTES_PER_MOTOR);
    if (hexmeow_canfd_send_fd(c->canfd_fd, HEXMEOW_RPDO_COB_ID, data,
                                total) != 0) {
        die_io("send RPDO (MIT)");
    }
}

static void build_and_send_rpdo_velocity(hexmeow_ctrl_t *c) {
    uint8_t  data[HEXMEOW_MAX_MOTORS * HEXMEOW_VELOCITY_BYTES_PER_MOTOR] = {0};
    uint8_t  cnt = c->set->count;

    pthread_mutex_lock(&c->lock);
    for (uint8_t i = 0; i < cnt; i++) {
        uint16_t torque = c->vel_torque_permille[i];
        float    target = c->vel_target_rev_s[i];
        uint8_t *p = &data[i * HEXMEOW_VELOCITY_BYTES_PER_MOTOR];
        p[0] = (uint8_t)(torque & 0xFF);
        p[1] = (uint8_t)((torque >> 8) & 0xFF);
        memcpy(&p[2], &target, sizeof(float));
    }
    pthread_mutex_unlock(&c->lock);

    uint8_t total = (uint8_t)(cnt * HEXMEOW_VELOCITY_BYTES_PER_MOTOR);
    if (hexmeow_canfd_send_fd(c->canfd_fd, HEXMEOW_RPDO_COB_ID, data,
                                total) != 0) {
        die_io("send RPDO (velocity)");
    }
}

/* --- thread main ------------------------------------------------------- */

static void *thread_main(void *arg) {
    hexmeow_ctrl_t *c = (hexmeow_ctrl_t *)arg;

    /* `last_hb_ms = 0` makes the first iteration's HB-due check fire
     * immediately, so the very first frame this thread emits is a
     * heartbeat -- before any RPDO. */
    uint64_t last_hb_ms = 0;
    uint64_t next_tick_ms = now_ms();
    uint32_t rpdo_count = 0;

    if (hexmeow_verbose) {
        fprintf(stderr,
                "[v] ctrl thread started; HB period=%u ms, RPDO period=%u ms\n",
                (unsigned)HEXMEOW_MASTER_HB_PERIOD_MS,
                (unsigned)HEXMEOW_RPDO_PERIOD_MS);
    }

    for (;;) {
        uint64_t t = now_ms();

        /* 1) Heartbeat first -- before any RPDO this tick. */
        if (t - last_hb_ms >= HEXMEOW_MASTER_HB_PERIOD_MS) {
            send_heartbeat(c);
            last_hb_ms = t;
        }

        /* 2) Shared RPDO. */
        if (c->set->mode == HEXMEOW_MODE_KIND_MIT) {
            build_and_send_rpdo_mit(c);
        } else {
            build_and_send_rpdo_velocity(c);
        }
        rpdo_count++;
        if (hexmeow_verbose && (rpdo_count % 50u) == 0u) {
            /* Rate-limit RPDO logging: one line every ~100 ms instead of
             * 500 lines/sec. */
            fprintf(stderr, "[v] %llums RPDO sent (count=%u)\n",
                    (unsigned long long)t, (unsigned)rpdo_count);
        }

        /* 3) Drain any pending TPDOs (non-blocking). */
        drain_rx(c);

        /* 4) Next tick at fixed RPDO cadence. */
        next_tick_ms += HEXMEOW_RPDO_PERIOD_MS;
        int64_t sleep_ms = (int64_t)next_tick_ms - (int64_t)now_ms();
        if (sleep_ms > 0) {
            struct timespec ts = {
                .tv_sec  = sleep_ms / 1000,
                .tv_nsec = (long)(sleep_ms % 1000) * 1000000L,
            };
            nanosleep(&ts, NULL);
        } else {
            /* Falling behind -- realign rather than spinning. */
            if (hexmeow_verbose && sleep_ms < -HEXMEOW_RPDO_PERIOD_MS) {
                fprintf(stderr,
                        "[v] %llums ctrl loop fell behind by %lld ms\n",
                        (unsigned long long)now_ms(),
                        (long long)(-sleep_ms));
            }
            next_tick_ms = now_ms();
        }
    }
    return NULL;  /* unreachable */
}

void hexmeow_ctrl_start(hexmeow_ctrl_t *c) {
    if (pthread_create(&c->thread, NULL, thread_main, c) != 0) {
        fprintf(stderr, "[hexmeow_ctrl] pthread_create failed: %s\n",
                strerror(errno));
        exit(EXIT_FAILURE);
    }
}
