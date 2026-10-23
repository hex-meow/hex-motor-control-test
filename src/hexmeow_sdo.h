/*
 * Minimal CANopen SDO master / client.
 *
 * Only expedited transfers (data length 1..4 bytes) are supported, which
 * is sufficient for every object touched during motor initialization. All
 * functions are synchronous and block until the response arrives or the
 * per-call timeout elapses.
 *
 * The client owns its own classic SocketCAN file descriptor; all SDO
 * traffic is independent of the high-speed CAN-FD control plane handled
 * by `hexmeow_canfd.h`.
 *
 * Error policy: functions return 0 on success and a negative error code on
 * failure (timeout, abort, transport error). Callers should print a
 * meaningful message and exit -- this demo does not retry or recover.
 */
#ifndef HEXMEOW_SDO_H
#define HEXMEOW_SDO_H

#include <stdint.h>

typedef struct hexmeow_sdo {
    int fd;
} hexmeow_sdo_t;

/* Negative return codes */
#define HEXMEOW_SDO_OK                 0
#define HEXMEOW_SDO_ERR_IO            -1
#define HEXMEOW_SDO_ERR_TIMEOUT       -2
#define HEXMEOW_SDO_ERR_ABORT         -3   /* server-side abort */
#define HEXMEOW_SDO_ERR_PROTOCOL      -4   /* unexpected response */
#define HEXMEOW_SDO_ERR_ARG           -5

/* Open a classic SocketCAN socket bound to `ifname` for SDO use.
 * Aborts on failure. */
void hexmeow_sdo_open(hexmeow_sdo_t *sdo, const char *ifname);

void hexmeow_sdo_close(hexmeow_sdo_t *sdo);

/* Expedited download (write) `len` bytes (1..4) to `idx`/`sub` on `node`.
 * Returns 0 on success, negative on error. On HEXMEOW_SDO_ERR_ABORT,
 * `*abort_code` (if non-NULL) is filled with the 32-bit abort code. */
int hexmeow_sdo_download(hexmeow_sdo_t *sdo, uint8_t node, uint16_t idx,
                           uint8_t sub, const void *data, uint8_t len,
                           int timeout_ms, uint32_t *abort_code);

/* Expedited upload (read) up to 4 bytes from `idx`/`sub` on `node`. On
 * success, `*out_len` is set to the actual byte count and the bytes are
 * written into `out` (which must hold at least 4 bytes). */
int hexmeow_sdo_upload(hexmeow_sdo_t *sdo, uint8_t node, uint16_t idx,
                         uint8_t sub, void *out, uint8_t *out_len,
                         int timeout_ms, uint32_t *abort_code);

/* Convenience helpers (download). All return 0 on success. */
int hexmeow_sdo_dl_u8 (hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint8_t  v);
int hexmeow_sdo_dl_u16(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint16_t v);
int hexmeow_sdo_dl_u32(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint32_t v);
int hexmeow_sdo_dl_i8 (hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, int8_t   v);
int hexmeow_sdo_dl_f32(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, float    v);

/* Convenience helpers (upload). */
int hexmeow_sdo_ul_u8 (hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint8_t  *v);
int hexmeow_sdo_ul_u16(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint16_t *v);
int hexmeow_sdo_ul_u32(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint32_t *v);
int hexmeow_sdo_ul_f32(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, float    *v);

const char *hexmeow_sdo_strerr(int rc);

#endif /* HEXMEOW_SDO_H */
