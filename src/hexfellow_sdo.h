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
 * by `hexfellow_canfd.h`.
 *
 * Error policy: functions return 0 on success and a negative error code on
 * failure (timeout, abort, transport error). Callers should print a
 * meaningful message and exit -- this demo does not retry or recover.
 */
#ifndef HEXFELLOW_SDO_H
#define HEXFELLOW_SDO_H

#include <stdint.h>

typedef struct hexfellow_sdo {
    int fd;
} hexfellow_sdo_t;

/* Negative return codes */
#define HEXFELLOW_SDO_OK                 0
#define HEXFELLOW_SDO_ERR_IO            -1
#define HEXFELLOW_SDO_ERR_TIMEOUT       -2
#define HEXFELLOW_SDO_ERR_ABORT         -3   /* server-side abort */
#define HEXFELLOW_SDO_ERR_PROTOCOL      -4   /* unexpected response */
#define HEXFELLOW_SDO_ERR_ARG           -5

/* Open a classic SocketCAN socket bound to `ifname` for SDO use.
 * Aborts on failure. */
void hexfellow_sdo_open(hexfellow_sdo_t *sdo, const char *ifname);

void hexfellow_sdo_close(hexfellow_sdo_t *sdo);

/* Expedited download (write) `len` bytes (1..4) to `idx`/`sub` on `node`.
 * Returns 0 on success, negative on error. On HEXFELLOW_SDO_ERR_ABORT,
 * `*abort_code` (if non-NULL) is filled with the 32-bit abort code. */
int hexfellow_sdo_download(hexfellow_sdo_t *sdo, uint8_t node, uint16_t idx,
                           uint8_t sub, const void *data, uint8_t len,
                           int timeout_ms, uint32_t *abort_code);

/* Expedited upload (read) up to 4 bytes from `idx`/`sub` on `node`. On
 * success, `*out_len` is set to the actual byte count and the bytes are
 * written into `out` (which must hold at least 4 bytes). */
int hexfellow_sdo_upload(hexfellow_sdo_t *sdo, uint8_t node, uint16_t idx,
                         uint8_t sub, void *out, uint8_t *out_len,
                         int timeout_ms, uint32_t *abort_code);

/* Convenience helpers (download). All return 0 on success. */
int hexfellow_sdo_dl_u8 (hexfellow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint8_t  v);
int hexfellow_sdo_dl_u16(hexfellow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint16_t v);
int hexfellow_sdo_dl_u32(hexfellow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint32_t v);
int hexfellow_sdo_dl_i8 (hexfellow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, int8_t   v);
int hexfellow_sdo_dl_f32(hexfellow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, float    v);

/* Convenience helpers (upload). */
int hexfellow_sdo_ul_u8 (hexfellow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint8_t  *v);
int hexfellow_sdo_ul_u16(hexfellow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint16_t *v);
int hexfellow_sdo_ul_u32(hexfellow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint32_t *v);
int hexfellow_sdo_ul_f32(hexfellow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, float    *v);

const char *hexfellow_sdo_strerr(int rc);

#endif /* HEXFELLOW_SDO_H */
