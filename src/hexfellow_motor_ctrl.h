/*
 * Hexfellow runtime control loop.
 *
 * Drives one CAN-FD socket on a 1ms cadence:
 *   - sends master heartbeat (cob 0x710) every 50ms
 *   - sends shared RPDO (cob 0x190) every 1ms
 *   - drains incoming TPDO1 / TPDO2 frames and updates per-motor state
 *
 * Targets are written by the application via `hexfellow_ctrl_set_*` and the
 * loop reads the latest values under a mutex.
 *
 * Lifecycle: this demo has no graceful shutdown path. The control thread
 * runs forever; the application is expected to exit on SIGINT via the
 * default signal action or an `_exit(0)` handler. The motor will detect
 * the heartbeat loss on its own and stop spinning -- there is no need to
 * disable the drive or send NMT pre-operational on shutdown.
 */
#ifndef HEXFELLOW_MOTOR_CTRL_H
#define HEXFELLOW_MOTOR_CTRL_H

#include <pthread.h>
#include <stdint.h>

#include "hexfellow_common.h"
#include "hexfellow_mit_target.h"
#include "hexfellow_motor_init.h"

typedef struct {
    /* Last-decoded TPDO1 fields. */
    float    position_rev;        /* 0x6064 (single-turn f32) */
    int32_t  multi_turns;         /* multi-turn extension computed locally */
    uint32_t timestamp_us;        /* 0x1013 timestamp from motor */
    int16_t  raw_torque_permille; /* 0x6077 */
    uint16_t error_code;          /* 0x603F */
    /* Last-decoded TPDO2 fields. */
    uint16_t status_word;         /* 0x6041 */
    int16_t  driver_temp_x10;     /* 0x2204-1 (0.1 C units) */
    int16_t  motor_temp_x10;      /* 0x2204-2 */
    uint16_t control_word;        /* 0x6040 (echo) */
    uint64_t last_tpdo1_ms;
    uint64_t last_tpdo2_ms;
} hexfellow_motor_state_t;

typedef struct {
    int                       canfd_fd;
    hexfellow_motor_set_t    *set;            /* not owned */

    /* Targets (protected by mutex). */
    pthread_mutex_t           lock;
    hexfellow_mit_target_t    mit_targets[HEXFELLOW_MAX_MOTORS];
    /* For velocity mode: */
    float                     vel_target_rev_s[HEXFELLOW_MAX_MOTORS];
    uint16_t                  vel_torque_permille[HEXFELLOW_MAX_MOTORS];

    /* Per-motor decoded state. */
    hexfellow_motor_state_t   state[HEXFELLOW_MAX_MOTORS];

    /* Single thread carrying both the master heartbeat (every 50 ms) and
     * the shared RPDO (every HEXFELLOW_RPDO_PERIOD_MS). Coupling them is
     * intentional: if the RPDO send path ever wedges, the heartbeat
     * stops too, the motor's 250 ms watchdog trips, and the failure is
     * visible -- rather than the motor happily spinning on a stale
     * target. */
    pthread_t                 thread;
} hexfellow_ctrl_t;

/* Initialize control state. All targets default to zero -- no rotation will
 * be commanded until the application calls one of the `set_*` helpers. */
void hexfellow_ctrl_init(hexfellow_ctrl_t *c, hexfellow_motor_set_t *set, int canfd_fd);

/* Spawn the control thread.
 *
 * The thread runs a tight `HB-if-due → RPDO → drain → sleep` loop on a
 * `HEXFELLOW_RPDO_PERIOD_MS` (= 2 ms) cadence. Master heartbeats are
 * emitted every 50 ms from inside that same loop. The thread runs for
 * the lifetime of the process; there is no `_stop` / `_join`. */
void hexfellow_ctrl_start(hexfellow_ctrl_t *c);

/* Update target for a single motor in MIT mode. */
void hexfellow_ctrl_set_mit_target(hexfellow_ctrl_t *c, uint8_t index,
                                   const hexfellow_mit_target_t *t);

/* Update target for a single motor in velocity mode. */
void hexfellow_ctrl_set_velocity_target(hexfellow_ctrl_t *c, uint8_t index,
                                        float target_rev_s,
                                        uint16_t torque_permille);

/* Snapshot the latest state (caller-allocated `out`). */
void hexfellow_ctrl_snapshot(hexfellow_ctrl_t *c, uint8_t index,
                             hexfellow_motor_state_t *out);

#endif /* HEXFELLOW_MOTOR_CTRL_H */
