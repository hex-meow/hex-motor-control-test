#include "hexmeow_motor_init.h"
#include "hexmeow_canfd.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* ----- error helpers ---------------------------------------------------- */

static void die(const char *fmt, ...) __attribute__((noreturn, format(printf, 1, 2)));
static void die(const char *fmt, ...) {
    va_list ap;
    fprintf(stderr, "[hexmeow_motor_init] ERROR: ");
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

/* ----- public NMT helper ------------------------------------------------ */

int hexmeow_send_nmt(int fd, uint8_t cs, uint8_t node) {
    uint8_t data[2] = {cs, node};
    return hexmeow_canfd_send_classic(fd, HEXMEOW_COB_NMT, data, 2);
}

/* ----- TPDO mapping (same for every motor) ------------------------------ */

static void configure_tpdo1(hexmeow_sdo_t *sdo, uint8_t id) {
    /* Disable, clear mapping, write 4 entries, set comm params, enable. */
    uint32_t disable = 0xC0000180u | id;
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1800, 1, disable));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, 0x1A00, 0, 0));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1A00, 1, 0x60640020u)); /* pos f32 */
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1A00, 2, 0x10130020u)); /* timestamp u32 */
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1A00, 3, 0x60770010u)); /* torque i16 */
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1A00, 4, 0x603F0010u)); /* err code u16 */
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, 0x1A00, 0, 4));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, 0x1800, 2, 255));            /* type 255 */
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x1800, 3, HEXMEOW_TPDO1_INHIBIT_X100US));
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x1800, 5, HEXMEOW_TPDO1_EVENT_MS));
    uint32_t enable = 0x40000180u | id;
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1800, 1, enable));
}

static void configure_tpdo2(hexmeow_sdo_t *sdo, uint8_t id) {
    uint32_t disable = 0xC0000280u | id;
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1801, 1, disable));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, 0x1A01, 0, 0));
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1A01, 1, 0x60410010u)); /* status word */
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1A01, 2, 0x22040110u)); /* drv temp */
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1A01, 3, 0x22040210u)); /* mtr temp */
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1A01, 4, 0x60400010u)); /* control word */
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1A01, 5, 0x603F0010u)); /* err code */
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, 0x1A01, 0, 5));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, 0x1801, 2, 255));
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x1801, 3, HEXMEOW_TPDO2_INHIBIT_X100US));
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x1801, 5, HEXMEOW_TPDO2_EVENT_MS));
    uint32_t enable = 0x40000280u | id;
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1801, 1, enable));
}

/* ----- RPDO mapping helpers --------------------------------------------- */

/* Encode an OD entry as [bit-length][subindex][index_lo][index_hi]
 * The 32-bit value laid out little-endian on the wire is:
 *   ((index << 16) | (subindex << 8) | bitlen). */
static uint32_t od_entry(uint16_t index, uint8_t sub, uint8_t bitlen) {
    return ((uint32_t)index << 16) | ((uint32_t)sub << 8) | bitlen;
}

/* MIT padding object: 0x3000-0x03 Word (u32, 32 bits). */
#define PAD_U32  od_entry(0x3000, 0x03, 0x20)
#define PAD_U16  od_entry(0x3000, 0x02, 0x10)

static void rpdo_disable(hexmeow_sdo_t *sdo, uint8_t id) {
    uint32_t disable = 0x80000000u | HEXMEOW_RPDO_COB_ID;
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1400, 1, disable));
    SDO_DL(hexmeow_sdo_dl_u8 (sdo, id, 0x1400, 2, 255));
}

static void rpdo_enable(hexmeow_sdo_t *sdo, uint8_t id) {
    uint32_t enable = HEXMEOW_RPDO_COB_ID;
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1400, 1, enable));
}

/* For the MIT one-to-many layout, every motor occupies 8 bytes. Motor at
 * `index` writes its own `2004h-02` (low 32) + `2004h-03` (high 32);
 * everything else is filled with 32-bit padding objects. */
static void rpdo_map_mit(hexmeow_sdo_t *sdo, uint8_t id, uint8_t index,
                         uint8_t total) {
    SDO_DL(hexmeow_sdo_dl_u8(sdo, id, 0x1600, 0, 0));
    uint8_t sub = 1;
    for (uint8_t i = 0; i < total; i++) {
        if (i == index) {
            SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1600, sub++,
                                        od_entry(0x2004, 0x02, 0x20)));
            SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1600, sub++,
                                        od_entry(0x2004, 0x03, 0x20)));
        } else {
            SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1600, sub++, PAD_U32));
            SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1600, sub++, PAD_U32));
        }
    }
    SDO_DL(hexmeow_sdo_dl_u8(sdo, id, 0x1600, 0, (uint8_t)(total * 2)));
}

/* For the velocity layout, every motor occupies 6 bytes:
 *  [u16 6072h max-torque][f32 60FFh target-velocity].
 * Motor at `index` maps its own 6072h+60FFh; the others use padding
 * (u16 + u32 = 6 bytes). */
static void rpdo_map_velocity(hexmeow_sdo_t *sdo, uint8_t id, uint8_t index,
                              uint8_t total) {
    SDO_DL(hexmeow_sdo_dl_u8(sdo, id, 0x1600, 0, 0));
    uint8_t sub = 1;
    for (uint8_t i = 0; i < total; i++) {
        if (i == index) {
            SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1600, sub++,
                                        od_entry(0x6072, 0x00, 0x10)));
            SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1600, sub++,
                                        od_entry(0x60FF, 0x00, 0x20)));
        } else {
            SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1600, sub++, PAD_U16));
            SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1600, sub++, PAD_U32));
        }
    }
    SDO_DL(hexmeow_sdo_dl_u8(sdo, id, 0x1600, 0, (uint8_t)(total * 2)));
}

/* ----- per-motor init --------------------------------------------------- */

static void init_one_motor(hexmeow_motor_set_t *set, hexmeow_sdo_t *sdo,
                           int fd, uint8_t index) {
    uint8_t id = set->motors[index].node_id;
    fprintf(stderr, "[hexmeow] initializing motor #%u (CANopen id %u)\n",
            (unsigned)index, (unsigned)id);

    /* 1) NMT -> Pre-Operational so we can SDO-configure the motor. */
    if (hexmeow_send_nmt(fd, HEXMEOW_NMT_PRE_OPERATIONAL, id) != 0) {
        die("NMT pre-operational send failed for id=%u: %s", id, strerror(errno));
    }
    msleep(50);

    /* 2) Validate factory UID. */
    uint32_t factory_uid = 0;
    SDO_UL(hexmeow_sdo_ul_u32(sdo, id, 0x1018, 1, &factory_uid));
    if (factory_uid != HEXMEOW_FACTORY_UID) {
        die("motor id=%u factory UID 0x%08X != expected 0x%08X "
            "(is this really a hexmeow motor?)",
            id, factory_uid, HEXMEOW_FACTORY_UID);
    }

    /* 3) Validate firmware version (must be within [min, max]). */
    uint32_t fw = 0;
    SDO_UL(hexmeow_sdo_ul_u32(sdo, id, 0x1018, 3, &fw));
    if (fw < HEXMEOW_MIN_VERSION || fw > HEXMEOW_MAX_VERSION) {
        die("motor id=%u firmware version %u is outside range [%u, %u]",
            id, fw, HEXMEOW_MIN_VERSION, HEXMEOW_MAX_VERSION);
    }
    set->motors[index].firmware_version = fw;

    /* 4) Read serial number and peak torque (constants). */
    uint32_t serial = 0;
    SDO_UL(hexmeow_sdo_ul_u32(sdo, id, 0x1018, 4, &serial));
    set->motors[index].serial_number = serial;

    float peak = 0.0f;
    SDO_UL(hexmeow_sdo_ul_f32(sdo, id, 0x6076, 0, &peak));
    set->motors[index].peak_torque_mNm = peak;

    /* 5) Reset CiA402 state machine and clear faults. */
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x6040, 0, 0x0000));
    msleep(10);
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x6040, 0, 0x0080));   /* fault reset */
    msleep(10);
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x6040, 0, 0x0080));   /* fault reset (again) */
    msleep(10);

    /* 6) Disable consumer-heartbeat monitoring while we reconfigure. */
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1016, 1, 0));

    /* 7) Short-circuit-brake on disable. */
    SDO_DL(hexmeow_sdo_dl_u8(sdo, id, 0x2040, 0, 1));

    /* 8) Per-mode max-torque (6072h). */
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x6072, 0, set->torque_permille));

    /* 9) MIT-specific: KP/KD output torque limit and target ranges. */
    if (set->mode == HEXMEOW_MODE_KIND_MIT) {
        const hexmeow_mit_mapping_t *m = &set->mapping;
        SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x2004, 0x0E,
                                    set->kp_kd_torque_permille));
        SDO_DL(hexmeow_sdo_dl_f32(sdo, id, 0x2004, 0x04, m->position_min));
        SDO_DL(hexmeow_sdo_dl_f32(sdo, id, 0x2004, 0x05, m->position_max));
        SDO_DL(hexmeow_sdo_dl_f32(sdo, id, 0x2004, 0x06, m->velocity_min));
        SDO_DL(hexmeow_sdo_dl_f32(sdo, id, 0x2004, 0x07, m->velocity_max));
        SDO_DL(hexmeow_sdo_dl_f32(sdo, id, 0x2004, 0x08, m->kp_min));
        SDO_DL(hexmeow_sdo_dl_f32(sdo, id, 0x2004, 0x09, m->kp_max));
        SDO_DL(hexmeow_sdo_dl_f32(sdo, id, 0x2004, 0x0A, m->kd_min));
        SDO_DL(hexmeow_sdo_dl_f32(sdo, id, 0x2004, 0x0B, m->kd_max));
        SDO_DL(hexmeow_sdo_dl_f32(sdo, id, 0x2004, 0x0C, m->torque_min));
        SDO_DL(hexmeow_sdo_dl_f32(sdo, id, 0x2004, 0x0D, m->torque_max));

        /* Pre-load 2004h-02/03 with a zero-target value so the first frame
         * after enable doesn't cause an instantaneous torque spike. */
        hexmeow_mit_target_t zero = {0};
        uint8_t bytes[8];
        hexmeow_mit_target_pack(&zero, m, bytes);
        uint32_t lo = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                      ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
        uint32_t hi = (uint32_t)bytes[4] | ((uint32_t)bytes[5] << 8) |
                      ((uint32_t)bytes[6] << 16) | ((uint32_t)bytes[7] << 24);
        SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x2004, 0x02, lo));
        SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x2004, 0x03, hi));

        /* Enable compressed MIT control parameter. */
        SDO_DL(hexmeow_sdo_dl_u8(sdo, id, 0x2004, 0x01, 1));
    }

    /* 10) TPDO1/2 mapping. */
    configure_tpdo1(sdo, id);
    configure_tpdo2(sdo, id);

    /* 11) RPDO1: shared 0x190 + per-motor mapping. */
    rpdo_disable(sdo, id);
    if (set->mode == HEXMEOW_MODE_KIND_MIT) {
        rpdo_map_mit(sdo, id, index, set->count);
    } else {
        rpdo_map_velocity(sdo, id, index, set->count);
    }
    rpdo_enable(sdo, id);

    /* 12) Re-enable consumer-heartbeat monitoring of the master. */
    uint32_t hb = ((uint32_t)HEXMEOW_MASTER_NODE_ID << 16) |
                  HEXMEOW_MOTOR_HB_TIMEOUT_MS;
    SDO_DL(hexmeow_sdo_dl_u32(sdo, id, 0x1016, 1, hb));

    /* 13) Mode of operation (MIT=5, Profile Velocity=3). */
    int8_t mode_value = (set->mode == HEXMEOW_MODE_KIND_MIT)
                            ? HEXMEOW_MODE_MIT
                            : HEXMEOW_MODE_PROFILE_VELOCITY;
    SDO_DL(hexmeow_sdo_dl_i8(sdo, id, 0x6060, 0, mode_value));

    /* 14) CiA402 state-machine ramp: 6 -> 7 -> 0xF (operation enabled). */
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x6040, 0, 0x0006));
    msleep(10);
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x6040, 0, 0x0007));
    msleep(10);
    SDO_DL(hexmeow_sdo_dl_u16(sdo, id, 0x6040, 0, 0x000F));
    msleep(10);

    /* 15) NMT -> Operational so PDOs start flowing. */
    if (hexmeow_send_nmt(fd, HEXMEOW_NMT_OPERATIONAL, id) != 0) {
        die("NMT operational send failed for id=%u: %s", id, strerror(errno));
    }

    fprintf(stderr,
            "[hexmeow] motor #%u id=%u OK (fw=%u serial=0x%08X peak=%.0f mNm)\n",
            (unsigned)index, (unsigned)id, fw, serial, peak);
}

void hexmeow_motor_set_init(hexmeow_motor_set_t *set,
                              hexmeow_sdo_t *sdo, int fd) {
    if (set->count == 0 || set->count > HEXMEOW_MAX_MOTORS) {
        die("invalid motor count %u (must be 1..%u)",
            (unsigned)set->count, (unsigned)HEXMEOW_MAX_MOTORS);
    }
    /* Convention: CANopen IDs must start at 1 and be sequential. */
    for (uint8_t i = 0; i < set->count; i++) {
        set->motors[i].node_id = (uint8_t)(i + 1);
    }

    /* Stop everything first: broadcast pre-operational. */
    if (hexmeow_send_nmt(fd, HEXMEOW_NMT_PRE_OPERATIONAL, 0) != 0) {
        die("NMT pre-operational broadcast failed: %s", strerror(errno));
    }
    msleep(100);

    for (uint8_t i = 0; i < set->count; i++) {
        init_one_motor(set, sdo, fd, i);
    }
}

