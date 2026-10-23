#include "meow_motor_ctrl.h"
#include "hexmeow_canfd.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Global verbose flag (declared in meow_common.h), set by the demo main. */
int meow_verbose = 0;

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)(ts.tv_nsec / 1000000);
}

static void die_io(const char *what) {
    fprintf(stderr, "[meow_ctrl] FATAL: %s: %s\n", what, strerror(errno));
    exit(EXIT_FAILURE);
}

/* ----- conversions ------------------------------------------------------ */

uint16_t meow_gain_to_raw(float gain, float kp_kd_factor) {
    if (!isfinite(gain) || !isfinite(kp_kd_factor) || kp_kd_factor <= 0.0f) {
        return 0;
    }
    float raw = roundf(gain / kp_kd_factor);
    if (raw <= 0.0f)      return 0;
    if (raw >= 65535.0f)  return 65535;
    return (uint16_t)raw;
}

/* ----- wire encoding ---------------------------------------------------- */

/* CANopen is little-endian on the wire; serialize explicitly so a
 * big-endian cross build stays correct. */
static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_f32(uint8_t *p, float v) {
    uint32_t bits;
    memcpy(&bits, &v, 4);
    p[0] = (uint8_t)(bits & 0xFFu);
    p[1] = (uint8_t)((bits >> 8) & 0xFFu);
    p[2] = (uint8_t)((bits >> 16) & 0xFFu);
    p[3] = (uint8_t)((bits >> 24) & 0xFFu);
}

void meow_mit_target_pack(const meow_mit_target_t *t,
                          meow_stream_layout_t layout, uint8_t *out) {
    if (layout == MEOW_STREAM_SLIM) {
        /* 0x4102 :02 :03 :05 :06  -- 12 bytes, exactly an FD DLC. */
        memset(out, 0, MEOW_RPDO_LEN_SLIM);
        put_f32(out + 0,  t->vdes_rev_s);
        put_f32(out + 4,  t->tff_nm);
        put_u16(out + 8,  t->kd_raw);
        put_u16(out + 10, t->limit_permille);
        return;
    }
    /* 0x4102 :01 .. :06 plus a two-byte 0x3000:02 filler -- 20 bytes, again
     * exactly an FD DLC, so the frame length matches the mapping. */
    memset(out, 0, MEOW_RPDO_LEN_FULL);
    put_f32(out + 0,  t->pdes_rev);
    put_f32(out + 4,  t->vdes_rev_s);
    put_f32(out + 8,  t->tff_nm);
    put_u16(out + 12, t->kp_raw);
    put_u16(out + 14, t->kd_raw);
    put_u16(out + 16, t->limit_permille);
    /* out[18..20) stays zero: the filler object. */
}

/* ----- public state accessors ------------------------------------------- */

void meow_ctrl_init(meow_ctrl_t *c, meow_motor_t *motor, int canfd_fd) {
    memset(c, 0, sizeof(*c));
    c->canfd_fd = canfd_fd;
    c->motor = motor;
    pthread_mutex_init(&c->lock, NULL);
    /* target is all-zero after the memset: Kp = Kd = limit = Tff = 0 makes
     * the MIT torque law output exactly zero. */
}

void meow_ctrl_set_target(meow_ctrl_t *c, const meow_mit_target_t *t) {
    pthread_mutex_lock(&c->lock);
    c->target = *t;
    pthread_mutex_unlock(&c->lock);
}

void meow_ctrl_snapshot(meow_ctrl_t *c, meow_motor_state_t *out_state,
                        uint32_t *out_rpdo_count) {
    pthread_mutex_lock(&c->lock);
    if (out_state) *out_state = c->state;
    if (out_rpdo_count) *out_rpdo_count = c->rpdo_count;
    pthread_mutex_unlock(&c->lock);
}

/* ----- TPDO decoding ---------------------------------------------------- */

/* 0x4564 is a signed Q8.24 fixed-point count of revolutions: the value is
 * already multi-turn over +-128 Rev, so there is no wrap to accumulate. */
static float q8_24_to_rev(const uint8_t *p) {
    int32_t raw = (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                            ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
    return (float)raw / 16777216.0f;
}

static void handle_tpdo1(meow_ctrl_t *c, const uint8_t *d) {
    float    pos = q8_24_to_rev(d);
    uint16_t ts  = (uint16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8));
    int16_t  trq = (int16_t)((uint16_t)d[6] | ((uint16_t)d[7] << 8));

    pthread_mutex_lock(&c->lock);
    c->state.position_rev = pos;
    c->state.timestamp_us = ts;
    c->state.torque_permille = trq;
    c->state.last_tpdo1_ms = now_ms();
    c->state.tpdo1_count++;
    pthread_mutex_unlock(&c->lock);
}

static void handle_tpdo2(meow_ctrl_t *c, const uint8_t *d) {
    uint8_t  mode  = d[0];
    uint16_t err   = (uint16_t)((uint16_t)d[1] | ((uint16_t)d[2] << 8));
    int16_t  drv_t = (int16_t)((uint16_t)d[3] | ((uint16_t)d[4] << 8));
    int16_t  mtr_t = (int16_t)((uint16_t)d[5] | ((uint16_t)d[6] << 8));

    pthread_mutex_lock(&c->lock);
    c->state.mode_display = mode;
    c->state.error_code = err;
    c->state.driver_temp_x10 = drv_t;
    c->state.motor_temp_x10 = mtr_t;
    c->state.last_tpdo2_ms = now_ms();
    c->state.tpdo2_count++;
    pthread_mutex_unlock(&c->lock);
}

/* Drain the kernel RX queue. Non-blocking, so this returns the moment the
 * queue is empty and never delays the next RPDO. */
static void drain_rx(meow_ctrl_t *c) {
    uint16_t tpdo1_cob = (uint16_t)(MEOW_COB_TPDO1 | c->motor->node_id);
    uint16_t tpdo2_cob = (uint16_t)(MEOW_COB_TPDO2 | c->motor->node_id);
    int n_t1 = 0, n_t2 = 0, n_other = 0;

    for (;;) {
        struct canfd_frame f;
        int is_fd = 0;
        int rc = hexmeow_canfd_recv(c->canfd_fd, &f, &is_fd);
        if (rc == -2) break;              /* queue empty */
        if (rc == -1) {
            fprintf(stderr, "[meow_ctrl] WARN: recv failed: %s\n", strerror(errno));
            break;
        }
        uint16_t cob = (uint16_t)(f.can_id & 0x7FFu);
        if (cob == tpdo1_cob && f.len >= MEOW_TPDO1_LEN) {
            handle_tpdo1(c, f.data);
            n_t1++;
        } else if (cob == tpdo2_cob && f.len >= MEOW_TPDO2_LEN) {
            handle_tpdo2(c, f.data);
            n_t2++;
        } else {
            n_other++;
            continue;   /* not ours -- do not count it as FD evidence */
        }
        pthread_mutex_lock(&c->lock);
        if (is_fd) c->state.rx_fd++; else c->state.rx_classic++;
        pthread_mutex_unlock(&c->lock);
    }

    if (meow_verbose && (n_t1 | n_t2 | n_other)) {
        fprintf(stderr, "[v] %llums drain: T1=%d T2=%d other=%d\n",
                (unsigned long long)now_ms(), n_t1, n_t2, n_other);
    }
}

/* ----- TX --------------------------------------------------------------- */

static void send_heartbeat(meow_ctrl_t *c) {
    uint8_t  hb  = 0x05;   /* Operational */
    uint16_t cob = MEOW_COB_HB | MEOW_HOST_NODE_ID;
    if (hexmeow_canfd_send_classic(c->canfd_fd, cob, &hb, 1) != 0) {
        die_io("send heartbeat");
    }
}

static void send_rpdo(meow_ctrl_t *c) {
    uint8_t data[MEOW_RPDO_LEN_FULL];
    uint8_t len = meow_rpdo_len(c->motor->layout);

    pthread_mutex_lock(&c->lock);
    meow_mit_target_pack(&c->target, c->motor->layout, data);
    pthread_mutex_unlock(&c->lock);

    uint16_t cob = (uint16_t)(MEOW_COB_RPDO1 | c->motor->node_id);
    /* Always an FD frame: both layouts exceed 8 bytes, and the helper sets
     * BRS. A classic-CAN-only link fails the write here rather than silently
     * truncating -- which is exactly the failure this demo is meant to
     * surface. */
    if (hexmeow_canfd_send_fd(c->canfd_fd, cob, data, len) != 0) {
        die_io("send RPDO1 (CAN-FD)");
    }

    pthread_mutex_lock(&c->lock);
    c->rpdo_count++;
    pthread_mutex_unlock(&c->lock);
}

/* ----- thread ----------------------------------------------------------- */

static void *thread_main(void *arg) {
    meow_ctrl_t *c = (meow_ctrl_t *)arg;

    /* last_hb_ms = 0 makes the first HB-due check fire immediately, so the
     * very first frame this thread emits is a heartbeat, before any RPDO. */
    uint64_t last_hb_ms = 0;
    uint64_t next_tick_ms = now_ms();

    if (meow_verbose) {
        fprintf(stderr, "[v] meow ctrl thread started; HB=%u ms RPDO=%u ms "
                        "payload=%u B\n",
                (unsigned)MEOW_HOST_HB_PERIOD_MS,
                (unsigned)MEOW_RPDO_PERIOD_MS,
                (unsigned)meow_rpdo_len(c->motor->layout));
    }

    for (;;) {
        uint64_t t = now_ms();

        if (t - last_hb_ms >= MEOW_HOST_HB_PERIOD_MS) {
            send_heartbeat(c);
            last_hb_ms = t;
        }

        send_rpdo(c);
        drain_rx(c);

        next_tick_ms += MEOW_RPDO_PERIOD_MS;
        int64_t sleep_ms = (int64_t)next_tick_ms - (int64_t)now_ms();
        if (sleep_ms > 0) {
            struct timespec ts = {
                .tv_sec  = sleep_ms / 1000,
                .tv_nsec = (long)(sleep_ms % 1000) * 1000000L,
            };
            nanosleep(&ts, NULL);
        } else {
            if (meow_verbose && sleep_ms < -(int64_t)MEOW_RPDO_PERIOD_MS) {
                fprintf(stderr, "[v] %llums ctrl loop fell behind by %lld ms\n",
                        (unsigned long long)now_ms(), (long long)(-sleep_ms));
            }
            next_tick_ms = now_ms();
        }
    }
    return NULL;  /* unreachable */
}

void meow_ctrl_start(meow_ctrl_t *c) {
    if (pthread_create(&c->thread, NULL, thread_main, c) != 0) {
        fprintf(stderr, "[meow_ctrl] pthread_create failed: %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }
}
