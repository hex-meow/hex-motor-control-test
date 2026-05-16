/*
 * Hexfellow motor SDO initialization.
 *
 * All motors on a given CAN bus share a common RPDO COB-ID (0x190) and
 * pack their per-motor control bytes side-by-side into one CAN-FD
 * frame.
 *
 * Two flavors are provided:
 *
 *   - MIT mode (mode 5):       8 bytes / motor   (max 8 motors -> 64 bytes)
 *   - Profile velocity (mode 3): 6 bytes / motor (max 8 motors -> 48 bytes)
 *
 * Errors abort the program with an explanatory message; this is a demo
 * and does not implement any kind of recovery.
 */
#ifndef HEXFELLOW_MOTOR_INIT_H
#define HEXFELLOW_MOTOR_INIT_H

#include <stdint.h>
#include "hexfellow_common.h"
#include "hexfellow_mit_target.h"
#include "hexfellow_sdo.h"

typedef enum {
    HEXFELLOW_MODE_KIND_MIT      = 0,
    HEXFELLOW_MODE_KIND_VELOCITY = 1,
} hexfellow_mode_kind_t;

/* Per-motor information learned during initialization. */
typedef struct {
    uint8_t  node_id;            /* CANopen ID, 1..N */
    uint32_t firmware_version;   /* from 1018h-03 */
    uint32_t serial_number;      /* from 1018h-04 */
    float    peak_torque_mNm;    /* from 6076h (REAL32, mNm) */
} hexfellow_motor_runtime_t;

/* Configuration shared across all motors in a controlled set. */
typedef struct {
    hexfellow_mode_kind_t mode;
    uint8_t               count;                 /* 1..HEXFELLOW_MAX_MOTORS */
    uint16_t              torque_permille;       /* 6072h, 0..1000 */
    uint16_t              kp_kd_torque_permille; /* 2004h-0E, MIT only */
    hexfellow_mit_mapping_t mapping;             /* MIT only */
    hexfellow_motor_runtime_t motors[HEXFELLOW_MAX_MOTORS];
} hexfellow_motor_set_t;

/* Run the full initialization sequence for every motor in `set->motors[0..count]`.
 *
 * `sdo`     -- already-opened SDO client.
 * `fd`      -- raw CAN-FD socket used for NMT broadcasts.
 * Aborts on any failure (UID mismatch, version mismatch, SDO timeout, ...). */
void hexfellow_motor_set_init(hexfellow_motor_set_t *set,
                              hexfellow_sdo_t *sdo, int fd);

/* Send NMT command (CS, node) to a single node. node = 0 means broadcast. */
int hexfellow_send_nmt(int fd, uint8_t cs, uint8_t node);

#endif /* HEXFELLOW_MOTOR_INIT_H */
