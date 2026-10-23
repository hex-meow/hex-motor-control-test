/*
 * Shared constants for the Meow Motor (hexmeow 4310 / 4342) demo.
 *
 * The Meow Motor does NOT speak CiA402. It has its own object dictionary
 * (0x41xx / 0x45xx) and a single flat mode byte at 0x4401 instead of the
 * CiA402 state machine -- so none of the hexmeow_* control code applies.
 * Only the transport layers (hexmeow_canfd.*, hexmeow_sdo.*) are shared.
 *
 * References:
 *   - can-related-conventions/定制电机协议.md   (0x2100, 0x3000, 0x4102, 0x4103)
 *   - hex-meow-fw/motor-od.md                   (0x44xx, 0x45xx, 0x4713)
 */
#ifndef MEOW_COMMON_H
#define MEOW_COMMON_H

#include <stdint.h>

/* ---- Identity (0x1018) ------------------------------------------------- */

#define MEOW_VENDOR_ID          0x00686578u   /* ASCII-ish "hex" */
#define MEOW_PRODUCT_4310       0x6C64BC78u
#define MEOW_PRODUCT_4342       0x6C64BCAAu

/* ---- Host side --------------------------------------------------------- */

/* CANopen node id this program presents as. The motor's consumer-heartbeat
 * entry (0x1016:01) is pointed at it, so the motor stops if we go quiet. */
#define MEOW_HOST_NODE_ID           0x10u
#define MEOW_HOST_HB_PERIOD_MS      50u
#define MEOW_MOTOR_HB_TIMEOUT_MS    250u

/* ---- CiA301 COB-ID bases ----------------------------------------------- */

#define MEOW_COB_NMT            0x000u
#define MEOW_COB_TPDO1          0x180u
#define MEOW_COB_RPDO1          0x200u
#define MEOW_COB_TPDO2          0x280u
#define MEOW_COB_HB             0x700u

#define MEOW_NMT_OPERATIONAL        0x01u
#define MEOW_NMT_PRE_OPERATIONAL    0x80u

/* PDO communication / mapping parameter records. */
#define MEOW_OD_TPDO1_COMM      0x1800u
#define MEOW_OD_TPDO2_COMM      0x1801u
#define MEOW_OD_TPDO1_MAP       0x1A00u
#define MEOW_OD_TPDO2_MAP       0x1A01u
#define MEOW_OD_RPDO1_COMM      0x1400u
#define MEOW_OD_RPDO1_MAP       0x1600u
#define MEOW_OD_CONSUMER_HB     0x1016u
#define MEOW_OD_IDENTITY        0x1018u

/* ---- Meow Motor object dictionary --------------------------------------- */

/* 0x3000 -- PDO filler objects, used to pad a mapping to an exact length. */
#define MEOW_OD_FILLER          0x3000u
#define MEOW_FILLER_U8          0x01u
#define MEOW_FILLER_U16         0x02u
#define MEOW_FILLER_U32         0x03u

/* 0x4102 -- uncompressed MIT control record. The torque law is
 *
 *     tau = Tff + clamp( Kp*factor*(Pdes - p) + Kd*factor*(Vdes - v), +-limit )
 *
 * where `limit` (:06) is permille of peak torque and clamps ONLY the Kp/Kd
 * result -- Tff is added outside it. `factor` is :07, set at the factory.
 *
 * The published protocol table marks these sub-indexes as not PDO-mappable.
 * That is wrong: mapping :02/:03/:05/:06 into RPDO1 is accepted and applied
 * on hexmeow 4310 firmware (verified in string-test-gui's pull-cord loop).
 * This demo additionally maps :01 and :04; pass --slim to fall back to the
 * four verified fields if a particular firmware rejects the other two. */
#define MEOW_OD_MIT             0x4102u
#define MEOW_MIT_PDES           0x01u   /* REAL32,     Rev            */
#define MEOW_MIT_VDES           0x02u   /* REAL32,     Rev/s          */
#define MEOW_MIT_TFF            0x03u   /* REAL32,     Nm             */
#define MEOW_MIT_KP             0x04u   /* UNSIGNED16, dimensionless  */
#define MEOW_MIT_KD             0x05u   /* UNSIGNED16, dimensionless  */
#define MEOW_MIT_LIMIT          0x06u   /* UNSIGNED16, permille       */
#define MEOW_MIT_FACTOR         0x07u   /* REAL32,     Kp/Kd scale    */

#define MEOW_OD_TEMPERATURE     0x4104u /* :01 driver, :02 motor, INTEGER16 0.1C */
#define MEOW_OD_MODE            0x4401u /* UNSIGNED8  rw, see MEOW_MODE_*        */
#define MEOW_OD_MODE_DISPLAY    0x4402u /* UNSIGNED8  ro                         */
#define MEOW_OD_ERROR_CODE      0x453Fu /* UNSIGNED16 ro                         */
#define MEOW_OD_POSITION        0x4564u /* REAL32 slot, signed Q8.24 Rev         */
#define MEOW_OD_MAX_TORQUE      0x4572u /* UNSIGNED16 rw, permille of peak       */
#define MEOW_OD_PEAK_TORQUE     0x4576u /* REAL32 ro,  Nm                        */
#define MEOW_OD_TORQUE_ACTUAL   0x4577u /* INTEGER16 ro, permille of peak        */
#define MEOW_OD_TIMESTAMP       0x4713u /* UNSIGNED16 ro, us, wraps every ~65 ms */

/* 0x4401 command values. */
#define MEOW_MODE_DISABLED      0u      /* short-circuit brake       */
#define MEOW_MODE_PROFILE_POS   1u
#define MEOW_MODE_PROFILE_VEL   2u
#define MEOW_MODE_TORQUE        3u
#define MEOW_MODE_MIT           4u
#define MEOW_MODE_CLEAR_ERROR   0xFFu

/* 0x4402 display values >= 0xA1 are faults (0xA4 = heartbeat error). */
#define MEOW_DISPLAY_IS_ERROR(d)    ((d) >= 0xA1u)

/* ---- PDO shapes --------------------------------------------------------- */

/* TPDO1: Q8.24 position (4) + u16 timestamp (2) + i16 torque permille (2). */
#define MEOW_TPDO1_LEN              8
#define MEOW_TPDO1_INHIBIT_X100US   5u    /* 0.5 ms */
#define MEOW_TPDO1_EVENT_MS         1u

/* TPDO2: u8 mode display (1) + u16 error (2) + 2x i16 temperature (4). */
#define MEOW_TPDO2_LEN              7
#define MEOW_TPDO2_INHIBIT_X100US   190u  /* 19 ms */
#define MEOW_TPDO2_EVENT_MS         20u

/* RPDO1 payload, both layouts padded with 0x3000 fillers to land exactly on
 * a legal CAN-FD DLC -- an RPDO shorter than its mapping is discarded, and
 * matching exactly avoids relying on the receiver tolerating trailing bytes.
 *
 *   full: Pdes(4) Vdes(4) Tff(4) Kp(2) Kd(2) limit(2) + u16 filler = 20
 *   slim:          Vdes(4) Tff(4)       Kd(2) limit(2)             = 12
 *
 * Both are > 8 bytes, so both force a CAN-FD frame -- which is the point of
 * this demo. */
#define MEOW_RPDO_LEN_FULL          20
#define MEOW_RPDO_LEN_SLIM          12

/* RPDO send cadence. */
#define MEOW_RPDO_PERIOD_MS         2u

/* SDO timeout used during init (per transaction). */
#define MEOW_SDO_TIMEOUT_MS         500

/* Verbose logging toggle, set to 1 by `--verbose` / `-v`. */
extern int meow_verbose;

#endif /* MEOW_COMMON_H */
