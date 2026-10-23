/*
 * Meow Motor SDO bring-up for a single motor.
 *
 * Unlike the hexmeow demos this drives exactly one motor on its own
 * default RPDO1 COB-ID (0x200 + node_id) -- there is no one-to-many frame
 * to slice up, which keeps the mapping readable and makes the CAN-FD
 * payload obvious on a bus trace.
 *
 * Errors abort the program with an explanatory message; this is a demo and
 * does not implement any kind of recovery.
 */
#ifndef MEOW_MOTOR_INIT_H
#define MEOW_MOTOR_INIT_H

#include <stdint.h>

#include "hexmeow_sdo.h"
#include "meow_common.h"

typedef enum {
    /* All six 0x4102 sub-indexes are streamed (20-byte frame). */
    MEOW_STREAM_FULL = 0,
    /* Only the four sub-indexes proven mappable on 4310 firmware are
     * streamed (12-byte frame); Pdes and Kp are pinned over SDO at mode
     * entry and never move. */
    MEOW_STREAM_SLIM = 1,
} meow_stream_layout_t;

typedef struct {
    /* --- caller-provided --------------------------------------------- */
    uint8_t              node_id;             /* 1..127                  */
    meow_stream_layout_t layout;
    uint16_t             max_torque_permille; /* written to 0x4572       */

    /* --- learned during init ----------------------------------------- */
    uint32_t    vendor_id;        /* 0x1018:01 */
    uint32_t    product_code;     /* 0x1018:02 */
    uint32_t    revision;         /* 0x1018:03 */
    uint32_t    serial;           /* 0x1018:04 */
    const char *model_name;       /* looked up from the identity pair    */
    float       peak_torque_nm;   /* 0x4576                              */
    float       kp_kd_factor;     /* 0x4102:07, Nm per (unit * Rev)      */
} meow_motor_t;

/* Run the full bring-up: identity check, fault clear, disable, TPDO1/TPDO2
 * and RPDO1 mapping, consumer heartbeat, torque ceiling, MIT mode entry,
 * NMT operational. No host heartbeat may be emitted while this runs -- see
 * the note in meow_motor_init.c. Aborts on any failure. */
void meow_motor_init(meow_motor_t *m, hexmeow_sdo_t *sdo, int canfd_fd);

/* Send an NMT command to a single node (node = 0 broadcasts). */
int meow_send_nmt(int canfd_fd, uint8_t cs, uint8_t node);

/* Wire length of the RPDO1 payload for a layout. */
uint8_t meow_rpdo_len(meow_stream_layout_t layout);

/* Human-readable name for a 0x4402 mode-display byte. */
const char *meow_mode_display_str(uint8_t display);

#endif /* MEOW_MOTOR_INIT_H */
