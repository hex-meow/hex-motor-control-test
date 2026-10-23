/*
 * Meow Motor runtime control loop.
 *
 * One thread drives one CAN-FD socket:
 *   - host heartbeat (cob 0x700 + MEOW_HOST_NODE_ID) every 50 ms, classic CAN
 *   - RPDO1 (cob 0x200 + node_id) every MEOW_RPDO_PERIOD_MS, always CAN-FD
 *   - drains incoming TPDO1 / TPDO2 and updates the motor state
 *
 * Heartbeat and RPDO share the thread on purpose: if the RPDO write path
 * ever wedges, the heartbeat stops with it, the motor's 250 ms consumer
 * watchdog trips, and the motor brakes instead of spinning on a stale
 * target.
 *
 * Lifecycle: no graceful shutdown. The thread runs until the process exits;
 * the motor notices the heartbeat gap on its own.
 */
#ifndef MEOW_MOTOR_CTRL_H
#define MEOW_MOTOR_CTRL_H

#include <pthread.h>
#include <stdint.h>

#include "meow_common.h"
#include "meow_motor_init.h"

/* One MIT command, already in the raw units 0x4102 expects. */
typedef struct {
    float    pdes_rev;        /* :01 REAL32,     Rev            */
    float    vdes_rev_s;      /* :02 REAL32,     Rev/s          */
    float    tff_nm;          /* :03 REAL32,     Nm             */
    uint16_t kp_raw;          /* :04 UNSIGNED16, dimensionless  */
    uint16_t kd_raw;          /* :05 UNSIGNED16, dimensionless  */
    uint16_t limit_permille;  /* :06 UNSIGNED16, permille       */
} meow_mit_target_t;

typedef struct {
    /* TPDO1 */
    float    position_rev;        /* 0x4564, signed Q8.24, multi-turn +-128 */
    uint16_t timestamp_us;        /* 0x4713, wraps every ~65 ms             */
    int16_t  torque_permille;     /* 0x4577, permille of peak torque        */
    /* TPDO2 */
    uint8_t  mode_display;        /* 0x4402 */
    uint16_t error_code;          /* 0x453F */
    int16_t  driver_temp_x10;     /* 0x4104:01, 0.1 C */
    int16_t  motor_temp_x10;      /* 0x4104:02, 0.1 C */

    uint64_t last_tpdo1_ms;
    uint64_t last_tpdo2_ms;
    uint32_t tpdo1_count;
    uint32_t tpdo2_count;

    /* CAN-FD evidence: how the received PDOs actually arrived on the wire.
     * With the motor's BRS flag (0x2100:03) at its default of 1 every PDO
     * it sends is FD+BRS, so a non-zero `rx_classic` here means something
     * in the path is falling back to classic CAN. */
    uint32_t rx_fd;
    uint32_t rx_classic;
} meow_motor_state_t;

typedef struct {
    int                canfd_fd;
    meow_motor_t      *motor;       /* not owned */

    pthread_mutex_t    lock;
    meow_mit_target_t  target;
    meow_motor_state_t state;
    uint32_t           rpdo_count;

    pthread_t          thread;
} meow_ctrl_t;

/* Zero every field. The target starts at all-zero, which the MIT torque law
 * evaluates to exactly zero torque -- no motion until the caller sets one. */
void meow_ctrl_init(meow_ctrl_t *c, meow_motor_t *motor, int canfd_fd);

/* Spawn the control thread. The first frame it emits is a heartbeat. */
void meow_ctrl_start(meow_ctrl_t *c);

void meow_ctrl_set_target(meow_ctrl_t *c, const meow_mit_target_t *t);

/* Snapshot state and the RPDO send counter (either pointer may be NULL). */
void meow_ctrl_snapshot(meow_ctrl_t *c, meow_motor_state_t *out_state,
                        uint32_t *out_rpdo_count);

/* Physical -> raw conversions for the two gain fields. Both gains are
 * dimensionless on the wire and are scaled by 0x4102:07:
 *
 *   Kp_raw * factor * position_error(Rev)   = torque (Nm)
 *   Kd_raw * factor * velocity_error(Rev/s) = torque (Nm)
 *
 * Results saturate at 65535 rather than wrapping. */
uint16_t meow_gain_to_raw(float gain, float kp_kd_factor);

/* Pack a target into the wire layout for `layout`. `out` must hold at least
 * meow_rpdo_len(layout) bytes; filler bytes are zeroed. */
void meow_mit_target_pack(const meow_mit_target_t *t,
                          meow_stream_layout_t layout, uint8_t *out);

#endif /* MEOW_MOTOR_CTRL_H */
