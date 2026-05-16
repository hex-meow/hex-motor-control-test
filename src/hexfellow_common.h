/*
 * Shared constants for the hexfellow CANopen motor demos.
 */
#ifndef HEXFELLOW_COMMON_H
#define HEXFELLOW_COMMON_H

#include <stdint.h>

/* ---- Hard preset conditions (see README.md) ----------------------------- */

/* Master node ID for this controller. The motors are configured to monitor
 * heartbeats from this node. */
#define HEXFELLOW_MASTER_NODE_ID    0x10u

/* Shared RPDO COB-ID used for one-to-many control. Equal to the master's
 * default TPDO1 COB-ID (0x180 + 0x10). */
#define HEXFELLOW_RPDO_COB_ID       0x190u

/* Maximum number of motors on a single CAN bus. CAN-FD frame data length
 * is 64 bytes, so 8 motors * 8 bytes/motor = 64 bytes (MIT mode). */
#define HEXFELLOW_MAX_MOTORS        8

/* Vendor / firmware checks (1018h identity object). */
#define HEXFELLOW_FACTORY_UID       0x4859444Cu  /* factory UID reported by hexfellow firmware */
#define HEXFELLOW_REQUIRED_VERSION  8u           /* required firmware == 8 */

/* Heartbeat parameters. */
#define HEXFELLOW_MASTER_HB_PERIOD_MS    50u
#define HEXFELLOW_MOTOR_HB_TIMEOUT_MS    250u

/* TPDO event-timer / inhibit values (in 0.1ms units for inhibit, ms for event). */
#define HEXFELLOW_TPDO1_INHIBIT_X100US   5u    /* 0.5ms */
#define HEXFELLOW_TPDO1_EVENT_MS         1u
#define HEXFELLOW_TPDO2_INHIBIT_X100US   190u  /* 19ms */
#define HEXFELLOW_TPDO2_EVENT_MS         20u

/* Default torque limit (permille of peak torque) used by demo_mit and
 * demo_velocity unless the user overrides on CLI. */
#define HEXFELLOW_DEFAULT_TORQUE_PERMILLE   800u

/* Per-motor RPDO payload sizes for each demo flavor. */
#define HEXFELLOW_MIT_BYTES_PER_MOTOR        8   /* 2x u32 from 2004h-02 / 2004h-03 */
#define HEXFELLOW_VELOCITY_BYTES_PER_MOTOR   6   /* u16 6072h + f32 60FFh */

/* TPDO sizes (firmware always sends 12-byte FD frames). */
#define HEXFELLOW_TPDO1_LEN                 12
#define HEXFELLOW_TPDO2_LEN                 12

/* SDO timeout used during init (per-transaction). */
#define HEXFELLOW_SDO_TIMEOUT_MS            500

/* CiA301 default COB-IDs */
#define HEXFELLOW_COB_NMT       0x000u
#define HEXFELLOW_COB_TPDO1     0x180u
#define HEXFELLOW_COB_TPDO2     0x280u
#define HEXFELLOW_COB_RSDO      0x600u
#define HEXFELLOW_COB_TSDO      0x580u
#define HEXFELLOW_COB_HB        0x700u

/* NMT command bytes */
#define HEXFELLOW_NMT_OPERATIONAL       0x01u
#define HEXFELLOW_NMT_PRE_OPERATIONAL   0x80u
#define HEXFELLOW_NMT_RESET_NODE        0x81u
#define HEXFELLOW_NMT_RESET_COMM        0x82u

/* CiA402 modes-of-operation values */
#define HEXFELLOW_MODE_PROFILE_VELOCITY 3   /* 6060h = 3 (mode 3) */
#define HEXFELLOW_MODE_MIT              5   /* 6060h = 5 (MIT mode) */

/* RPDO send cadence (ms). 2 ms is plenty -- the firmware is happy with
 * anything significantly faster than the motor's internal control loop. */
#define HEXFELLOW_RPDO_PERIOD_MS        2u

/* Verbose logging toggle (extern). Set to 1 by passing `--verbose` /
 * `-v` on the demo command line. Used by every module that emits
 * "[v] ..." debug lines on stderr. */
extern int hexfellow_verbose;

#endif /* HEXFELLOW_COMMON_H */
