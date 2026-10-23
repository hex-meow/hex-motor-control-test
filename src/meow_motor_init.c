#include "meow_motor_init.h"
#include "hexmeow_canfd.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ----- error helpers ---------------------------------------------------- */

static void die(const char *fmt, ...) __attribute__((noreturn, format(printf, 1, 2)));
static void die(const char *fmt, ...) {
    va_list ap;
    fprintf(stderr, "[meow_motor_init] ERROR: ");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    exit(EXIT_FAILURE);
}

#define SDO_DL(call)                                                        \
    do {                                                                    \
        int _rc = (call);                                                   \
        if (_rc != 0) {                                                     \
            die("SDO download failed: %s (rc=%d) at %s",                    \
                hexmeow_sdo_strerr(_rc), _rc, #call);                     \
        }                                                                   \
    } while (0)

#define SDO_UL(call)                                                        \
    do {                                                                    \
        int _rc = (call);                                                   \
        if (_rc != 0) {                                                     \
            die("SDO upload failed: %s (rc=%d) at %s",                      \
                hexmeow_sdo_strerr(_rc), _rc, #call);                     \
        }                                                                   \
    } while (0)

static void msleep(int ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

/* ----- small helpers ---------------------------------------------------- */

int meow_send_nmt(int canfd_fd, uint8_t cs, uint8_t node) {
    uint8_t data[2] = {cs, node};
    return hexmeow_canfd_send_classic(canfd_fd, MEOW_COB_NMT, data, 2);
}

uint8_t meow_rpdo_len(meow_stream_layout_t layout) {
    return (layout == MEOW_STREAM_SLIM) ? MEOW_RPDO_LEN_SLIM
                                        : MEOW_RPDO_LEN_FULL;
}

const char *meow_mode_display_str(uint8_t display) {
    switch (display) {
    case 0x00: return "disabled";
    case 0x01: return "profile position";
    case 0x02: return "profile velocity";
    case 0x03: return "torque";
    case 0x04: return "MIT";
    case 0xA1: return "FAULT overcurrent";
    case 0xA2: return "FAULT overvoltage";
    case 0xA3: return "FAULT undervoltage";
    case 0xA4: return "FAULT heartbeat lost";
    case 0xA5: return "FAULT driver over-temperature";
    case 0xA6: return "FAULT motor over-temperature";
    case 0xAF: return "FAULT other (see 0x453F)";
    default:   return "unknown";
    }
}

static const char *model_name_for(uint32_t vendor, uint32_t product) {
    if (vendor != MEOW_VENDOR_ID) return NULL;
    if (product == MEOW_PRODUCT_4310) return "hexmeow 4310";
    if (product == MEOW_PRODUCT_4342) return "hexmeow 4342";
    return NULL;
}

/* PDO mapping entry: (index << 16) | (subindex << 8) | bit_length. */
static uint32_t od_entry(uint16_t index, uint8_t sub, uint8_t bitlen) {
    return ((uint32_t)index << 16) | ((uint32_t)sub << 8) | bitlen;
}

/* ----- TPDO mapping ----------------------------------------------------- */

/* TPDO1 = Q8.24 position | u16 timestamp | i16 torque permille  (8 bytes).
 * With 0x2100:03 (BRS) at its default of 1 the motor sends every PDO as
 * CAN-FD + BRS regardless of length, so this is also an FD receive path. */
static void configure_tpdo1(hexmeow_sdo_t *sdo, uint8_t id) {
    uint32_t cob = MEOW_COB_TPDO1 | id;
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_TPDO1_COMM, 1, 0xC0000000u | cob));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, MEOW_OD_TPDO1_MAP,  0, 0));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_TPDO1_MAP,  1, od_entry(MEOW_OD_POSITION,     0, 32)));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_TPDO1_MAP,  2, od_entry(MEOW_OD_TIMESTAMP,    0, 16)));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_TPDO1_MAP,  3, od_entry(MEOW_OD_TORQUE_ACTUAL,0, 16)));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, MEOW_OD_TPDO1_MAP,  0, 3));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, MEOW_OD_TPDO1_COMM, 2, 255));
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, MEOW_OD_TPDO1_COMM, 3, MEOW_TPDO1_INHIBIT_X100US));
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, MEOW_OD_TPDO1_COMM, 5, MEOW_TPDO1_EVENT_MS));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_TPDO1_COMM, 1, 0x40000000u | cob));
}

/* TPDO2 = u8 mode display | u16 error | i16 driver temp | i16 motor temp. */
static void configure_tpdo2(hexmeow_sdo_t *sdo, uint8_t id) {
    uint32_t cob = MEOW_COB_TPDO2 | id;
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_TPDO2_COMM, 1, 0xC0000000u | cob));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, MEOW_OD_TPDO2_MAP,  0, 0));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_TPDO2_MAP,  1, od_entry(MEOW_OD_MODE_DISPLAY, 0, 8)));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_TPDO2_MAP,  2, od_entry(MEOW_OD_ERROR_CODE,   0, 16)));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_TPDO2_MAP,  3, od_entry(MEOW_OD_TEMPERATURE,  1, 16)));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_TPDO2_MAP,  4, od_entry(MEOW_OD_TEMPERATURE,  2, 16)));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, MEOW_OD_TPDO2_MAP,  0, 4));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, MEOW_OD_TPDO2_COMM, 2, 255));
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, MEOW_OD_TPDO2_COMM, 3, MEOW_TPDO2_INHIBIT_X100US));
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, MEOW_OD_TPDO2_COMM, 5, MEOW_TPDO2_EVENT_MS));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_TPDO2_COMM, 1, 0x40000000u | cob));
}

/* ----- RPDO mapping ----------------------------------------------------- */

/* Write one 0x1600 mapping entry, reporting the SDO abort code on failure.
 *
 * This is the one step where a firmware difference is plausible: the vendor
 * protocol table claims no 0x4102 sub-index is PDO-mappable, and only
 * :02/:03/:05/:06 have been verified in the field. If :01 or :04 turn out to
 * be rejected on some build, the abort lands here and the message points at
 * --slim rather than leaving the user to decode 0x06040041 by hand. */
static void map_entry(hexmeow_sdo_t *sdo, uint8_t id, uint8_t sub,
                      uint16_t index, uint8_t obj_sub, uint8_t bitlen) {
    uint32_t value = od_entry(index, obj_sub, bitlen);
    /* Serialize explicitly rather than memcpy'ing host byte order -- CANopen
     * is little-endian on the wire and the rest of this repo never assumes
     * the host agrees. */
    uint8_t buf[4] = {
        (uint8_t)(value & 0xFFu),         (uint8_t)((value >> 8) & 0xFFu),
        (uint8_t)((value >> 16) & 0xFFu), (uint8_t)((value >> 24) & 0xFFu),
    };
    uint32_t abort_code = 0;
    int rc = hexmeow_sdo_download(sdo, id, MEOW_OD_RPDO1_MAP, sub, buf,
                                    4, MEOW_SDO_TIMEOUT_MS, &abort_code);
    if (rc == 0) return;
    if (rc == HEXMEOW_SDO_ERR_ABORT) {
        die("motor id=%u refused RPDO1 mapping entry %u -> 0x%04X:%02X "
            "(%u bits), SDO abort 0x%08X.\n"
            "       If this is 0x4102:01 or 0x4102:04, this firmware does not "
            "allow them in a PDO;\n"
            "       re-run with --slim to stream only 0x4102:02/:03/:05/:06.",
            id, sub, index, obj_sub, bitlen, abort_code);
    }
    die("motor id=%u RPDO1 mapping entry %u -> 0x%04X:%02X failed: %s (rc=%d)",
        id, sub, index, obj_sub, hexmeow_sdo_strerr(rc), rc);
}

/* RPDO1 carries the uncompressed MIT record. Both layouts are padded to an
 * exact CAN-FD DLC with a 0x3000 filler object so the frame length matches
 * the mapped length precisely (see meow_common.h). */
static void configure_rpdo1(hexmeow_sdo_t *sdo, uint8_t id,
                            meow_stream_layout_t layout) {
    uint32_t cob = MEOW_COB_RPDO1 | id;

    /* Disable first: a mapping may only be edited while the PDO is invalid. */
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_RPDO1_COMM, 1, 0x80000000u | cob));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, MEOW_OD_RPDO1_COMM, 2, 255)); /* async */
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, MEOW_OD_RPDO1_MAP,  0, 0));

    uint8_t count;
    if (layout == MEOW_STREAM_SLIM) {
        map_entry(sdo, id, 1, MEOW_OD_MIT, MEOW_MIT_VDES,  32);
        map_entry(sdo, id, 2, MEOW_OD_MIT, MEOW_MIT_TFF,   32);
        map_entry(sdo, id, 3, MEOW_OD_MIT, MEOW_MIT_KD,    16);
        map_entry(sdo, id, 4, MEOW_OD_MIT, MEOW_MIT_LIMIT, 16);
        count = 4;
    } else {
        map_entry(sdo, id, 1, MEOW_OD_MIT, MEOW_MIT_PDES,  32);
        map_entry(sdo, id, 2, MEOW_OD_MIT, MEOW_MIT_VDES,  32);
        map_entry(sdo, id, 3, MEOW_OD_MIT, MEOW_MIT_TFF,   32);
        map_entry(sdo, id, 4, MEOW_OD_MIT, MEOW_MIT_KP,    16);
        map_entry(sdo, id, 5, MEOW_OD_MIT, MEOW_MIT_KD,    16);
        map_entry(sdo, id, 6, MEOW_OD_MIT, MEOW_MIT_LIMIT, 16);
        /* Pad 18 -> 20 bytes, the next legal FD length. */
        map_entry(sdo, id, 7, MEOW_OD_FILLER, MEOW_FILLER_U16, 16);
        count = 7;
    }

    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, MEOW_OD_RPDO1_MAP,  0, count));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_RPDO1_COMM, 1, cob));
}

/* ----- bring-up --------------------------------------------------------- */

void meow_motor_init(meow_motor_t *m, hexmeow_sdo_t *sdo, int canfd_fd) {
    uint8_t id = m->node_id;
    if (id < 1 || id > 127) {
        die("node id must be 1..127 (got %u)", (unsigned)id);
    }
    if (m->max_torque_permille > 1000) {
        die("max torque must be 0..1000 permille (got %u)",
            (unsigned)m->max_torque_permille);
    }

    fprintf(stderr, "[meow] bringing up node %u (%s layout, RPDO1 cob 0x%03X, "
                    "%u-byte CAN-FD payload)\n",
            (unsigned)id,
            (m->layout == MEOW_STREAM_SLIM) ? "slim" : "full",
            (unsigned)(MEOW_COB_RPDO1 | id),
            (unsigned)meow_rpdo_len(m->layout));

    /* 1) Pre-Operational: PDOs stop, SDO still works. */
    if (meow_send_nmt(canfd_fd, MEOW_NMT_PRE_OPERATIONAL, id) != 0) {
        die("NMT pre-operational send failed for id=%u: %s", id, strerror(errno));
    }
    msleep(50);

    /* 2) Identity. A wrong vendor id means this is not a Meow Motor at all
     * and none of the object dictionary below exists -- hard stop. An
     * unrecognized product code under the right vendor is only a warning:
     * a newer model still speaks this protocol. */
    SDO_UL(hexmeow_sdo_ul_u32(sdo, id, MEOW_OD_IDENTITY, 1, &m->vendor_id));
    if (m->vendor_id != MEOW_VENDOR_ID) {
        die("node %u reports vendor id 0x%08X, expected 0x%08X -- this is not "
            "a Meow Motor", id, m->vendor_id, MEOW_VENDOR_ID);
    }
    SDO_UL(hexmeow_sdo_ul_u32(sdo, id, MEOW_OD_IDENTITY, 2, &m->product_code));
    SDO_UL(hexmeow_sdo_ul_u32(sdo, id, MEOW_OD_IDENTITY, 3, &m->revision));
    SDO_UL(hexmeow_sdo_ul_u32(sdo, id, MEOW_OD_IDENTITY, 4, &m->serial));
    m->model_name = model_name_for(m->vendor_id, m->product_code);
    if (m->model_name == NULL) {
        fprintf(stderr, "[meow] WARN: unrecognized product code 0x%08X; "
                        "continuing anyway\n", m->product_code);
        m->model_name = "unknown Meow Motor";
    }
    fprintf(stderr, "[meow] %s  rev=%u serial=0x%08X\n",
            m->model_name, m->revision, m->serial);

    /* 3) Clear a latched fault if there is one. The motor refuses every
     * other mode change until 0xFF has been accepted. */
    uint8_t display = 0;
    SDO_UL(hexmeow_sdo_ul_u8(sdo, id, MEOW_OD_MODE_DISPLAY, 0, &display));
    if (MEOW_DISPLAY_IS_ERROR(display)) {
        uint16_t detail = 0;
        (void)hexmeow_sdo_ul_u16(sdo, id, MEOW_OD_ERROR_CODE, 0, &detail);
        fprintf(stderr, "[meow] motor is in fault 0x%02X (%s, detail 0x%04X); "
                        "clearing\n",
                display, meow_mode_display_str(display), detail);
        SDO_DL(hexmeow_sdo_dl_u8(sdo, id, MEOW_OD_MODE, 0, MEOW_MODE_CLEAR_ERROR));
        msleep(20);
        SDO_UL(hexmeow_sdo_ul_u8(sdo, id, MEOW_OD_MODE_DISPLAY, 0, &display));
        if (MEOW_DISPLAY_IS_ERROR(display)) {
            die("fault 0x%02X (%s) did not clear -- fix the underlying cause "
                "before running this demo",
                display, meow_mode_display_str(display));
        }
    }

    /* 4) Disabled = windings shorted. Safe state while we reconfigure. */
    SDO_DL(hexmeow_sdo_dl_u8(sdo, id, MEOW_OD_MODE, 0, MEOW_MODE_DISABLED));

    /* 5) Factory scale factors. Both are needed to turn the physical units
     * this demo takes on the command line into the raw wire domain. */
    SDO_UL(hexmeow_sdo_ul_f32(sdo, id, MEOW_OD_PEAK_TORQUE, 0, &m->peak_torque_nm));
    SDO_UL(hexmeow_sdo_ul_f32(sdo, id, MEOW_OD_MIT, MEOW_MIT_FACTOR, &m->kp_kd_factor));
    if (!(m->peak_torque_nm > 0.0f)) {
        die("motor reports peak torque %.3f Nm (0x4576); refusing to command "
            "torque against an unknown scale", (double)m->peak_torque_nm);
    }
    if (!(m->kp_kd_factor > 0.0f)) {
        die("motor reports MIT Kp/Kd scale factor %.6f (0x4102:07); the demo "
            "cannot convert Kd into raw units", (double)m->kp_kd_factor);
    }
    fprintf(stderr, "[meow] peak torque %.3f Nm, MIT Kp/Kd factor %.6f\n",
            (double)m->peak_torque_nm, (double)m->kp_kd_factor);

    /* 6) PDO mapping. */
    configure_tpdo1(sdo, id);
    configure_tpdo2(sdo, id);
    configure_rpdo1(sdo, id, m->layout);

    /* 7) Motor-side torque ceiling, under everything the MIT law can ask
     * for -- including Tff, which the 0x4102:06 clamp does not cover. */
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, MEOW_OD_MAX_TORQUE, 0, m->max_torque_permille));

    /* 8) Arm the consumer-heartbeat watchdog LAST among the SDO writes.
     * The watchdog only starts counting once the motor has seen its first
     * host heartbeat, and this program deliberately emits none until the
     * control thread starts -- so configuring it early would be harmless,
     * but keeping it here mirrors the hexmeow demo and makes the "no
     * heartbeats during init" contract obvious. */
    uint32_t hb = ((uint32_t)MEOW_HOST_NODE_ID << 16) | MEOW_MOTOR_HB_TIMEOUT_MS;
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, MEOW_OD_CONSUMER_HB, 1, hb));

    /* 9) Zero every MIT field over SDO, then enter MIT mode. Writing all six
     * matters in --slim mode, where Pdes and Kp are never streamed and keep
     * these values for the whole run. With Kp = Kd = limit = Tff = 0 the
     * torque law outputs exactly zero, so entering the mode here commands no
     * motion even though PDOs are not flowing yet. */
    SDO_DL(hexmeow_sdo_dl_f32(sdo, id, MEOW_OD_MIT, MEOW_MIT_PDES,  0.0f));
    SDO_DL(hexmeow_sdo_dl_f32(sdo, id, MEOW_OD_MIT, MEOW_MIT_VDES,  0.0f));
    SDO_DL(hexmeow_sdo_dl_f32(sdo, id, MEOW_OD_MIT, MEOW_MIT_TFF,   0.0f));
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, MEOW_OD_MIT, MEOW_MIT_KP,    0));
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, MEOW_OD_MIT, MEOW_MIT_KD,    0));
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, MEOW_OD_MIT, MEOW_MIT_LIMIT, 0));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, MEOW_OD_MODE, 0, MEOW_MODE_MIT));

    /* 10) Operational: PDOs start flowing in both directions. */
    if (meow_send_nmt(canfd_fd, MEOW_NMT_OPERATIONAL, id) != 0) {
        die("NMT operational send failed for id=%u: %s", id, strerror(errno));
    }

    fprintf(stderr, "[meow] node %u in MIT mode, torque ceiling %u permille\n",
            (unsigned)id, (unsigned)m->max_torque_permille);
}
