#include "hexmeow_sdo.h"
#include "hexmeow_common.h"

#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

/* CCS bits (high nibble of byte 0)
 *  client download initiate:   0x20 | ((4-n) << 2) | 0x03   (expedited, size set)
 *  client upload   initiate:   0x40
 *  server download response:   0x60
 *  server upload   response:   0x40 | ((4-n) << 2) | 0x03   (expedited)
 *  abort:                      0x80
 */
#define SDO_CLI_DL_INITIATE_EXP(n)  (0x23u | (((4u - (uint8_t)(n)) & 0x3u) << 2))
#define SDO_CLI_UL_INITIATE         (0x40u)
#define SDO_SRV_DL_RESPONSE         (0x60u)
#define SDO_SRV_UL_RESPONSE_EXP     (0x40u)  /* high 3 bits = 010 */
#define SDO_ABORT                   (0x80u)

static int set_rcvto_ms(int fd, int ms) {
    struct timeval tv;
    tv.tv_sec = ms / 1000;
    tv.tv_usec = (ms % 1000) * 1000;
    return setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

void hexmeow_sdo_open(hexmeow_sdo_t *sdo, const char *ifname) {
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        fprintf(stderr, "[hexmeow_sdo] socket(PF_CAN) failed: %s\n",
                strerror(errno));
        exit(EXIT_FAILURE);
    }
    /* Filter only the SDO server response COB-ID range (0x580..0x5FF). */
    struct can_filter filt;
    filt.can_id = HEXMEOW_COB_TSDO;
    filt.can_mask = 0x780;  /* match 0x580..0x5FF */
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, sizeof(filt)) < 0) {
        fprintf(stderr, "[hexmeow_sdo] setsockopt(CAN_RAW_FILTER) failed: %s\n",
                strerror(errno));
        close(fd);
        exit(EXIT_FAILURE);
    }
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "[hexmeow_sdo] ioctl(SIOCGIFINDEX) for '%s' failed: %s\n",
                ifname, strerror(errno));
        close(fd);
        exit(EXIT_FAILURE);
    }
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "[hexmeow_sdo] bind '%s' failed: %s\n", ifname,
                strerror(errno));
        close(fd);
        exit(EXIT_FAILURE);
    }
    sdo->fd = fd;
}

void hexmeow_sdo_close(hexmeow_sdo_t *sdo) {
    if (sdo->fd >= 0) {
        close(sdo->fd);
        sdo->fd = -1;
    }
}

/* Drain any stale frames (e.g. a late response from a previous request). */
static void drain(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return;
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    struct can_frame f;
    while (read(fd, &f, sizeof(f)) > 0) { /* discard */ }
    fcntl(fd, F_SETFL, flags);
}

static int send_request(int fd, uint8_t node, const uint8_t cmd[8]) {
    struct can_frame f;
    memset(&f, 0, sizeof(f));
    f.can_id = (HEXMEOW_COB_RSDO | node) & CAN_SFF_MASK;
    f.can_dlc = 8;
    memcpy(f.data, cmd, 8);
    ssize_t n = write(fd, &f, sizeof(f));
    return (n == (ssize_t)sizeof(f)) ? 0 : -1;
}

static int wait_response(int fd, uint8_t node, uint16_t idx, uint8_t sub,
                         uint8_t out[8]) {
    /* Loop until we get a frame from this exact node with matching idx/sub,
     * or the socket times out. SO_RCVTIMEO is the per-read budget. */
    for (;;) {
        struct can_frame f;
        ssize_t n = read(fd, &f, sizeof(f));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return HEXMEOW_SDO_ERR_TIMEOUT;
            }
            return HEXMEOW_SDO_ERR_IO;
        }
        if (n != (ssize_t)sizeof(f) || f.can_dlc < 8) continue;
        uint16_t exp_id = (HEXMEOW_COB_TSDO | node);
        if ((f.can_id & CAN_SFF_MASK) != exp_id) continue;
        uint16_t resp_idx = (uint16_t)f.data[1] | ((uint16_t)f.data[2] << 8);
        uint8_t resp_sub = f.data[3];
        /* Aborts may carry the same idx/sub so they are still the right
         * frame to return to the caller. */
        if (resp_idx != idx || resp_sub != sub) continue;
        memcpy(out, f.data, 8);
        return HEXMEOW_SDO_OK;
    }
}

int hexmeow_sdo_download(hexmeow_sdo_t *sdo, uint8_t node, uint16_t idx,
                           uint8_t sub, const void *data, uint8_t len,
                           int timeout_ms, uint32_t *abort_code) {
    if (!sdo || sdo->fd < 0 || !data || len == 0 || len > 4) {
        return HEXMEOW_SDO_ERR_ARG;
    }
    if (set_rcvto_ms(sdo->fd, timeout_ms) < 0) return HEXMEOW_SDO_ERR_IO;
    drain(sdo->fd);
    uint8_t cmd[8] = {0};
    cmd[0] = SDO_CLI_DL_INITIATE_EXP(len);
    cmd[1] = (uint8_t)(idx & 0xFF);
    cmd[2] = (uint8_t)((idx >> 8) & 0xFF);
    cmd[3] = sub;
    memcpy(&cmd[4], data, len);
    if (send_request(sdo->fd, node, cmd) != 0) return HEXMEOW_SDO_ERR_IO;
    uint8_t resp[8];
    int rc = wait_response(sdo->fd, node, idx, sub, resp);
    if (rc != HEXMEOW_SDO_OK) return rc;
    if ((resp[0] & 0xE0) == SDO_ABORT) {
        if (abort_code) {
            *abort_code = (uint32_t)resp[4] | ((uint32_t)resp[5] << 8) |
                          ((uint32_t)resp[6] << 16) | ((uint32_t)resp[7] << 24);
        }
        return HEXMEOW_SDO_ERR_ABORT;
    }
    if (resp[0] != SDO_SRV_DL_RESPONSE) return HEXMEOW_SDO_ERR_PROTOCOL;
    return HEXMEOW_SDO_OK;
}

int hexmeow_sdo_upload(hexmeow_sdo_t *sdo, uint8_t node, uint16_t idx,
                         uint8_t sub, void *out, uint8_t *out_len,
                         int timeout_ms, uint32_t *abort_code) {
    if (!sdo || sdo->fd < 0 || !out || !out_len) return HEXMEOW_SDO_ERR_ARG;
    if (set_rcvto_ms(sdo->fd, timeout_ms) < 0) return HEXMEOW_SDO_ERR_IO;
    drain(sdo->fd);
    uint8_t cmd[8] = {0};
    cmd[0] = SDO_CLI_UL_INITIATE;
    cmd[1] = (uint8_t)(idx & 0xFF);
    cmd[2] = (uint8_t)((idx >> 8) & 0xFF);
    cmd[3] = sub;
    if (send_request(sdo->fd, node, cmd) != 0) return HEXMEOW_SDO_ERR_IO;
    uint8_t resp[8];
    int rc = wait_response(sdo->fd, node, idx, sub, resp);
    if (rc != HEXMEOW_SDO_OK) return rc;
    if ((resp[0] & 0xE0) == SDO_ABORT) {
        if (abort_code) {
            *abort_code = (uint32_t)resp[4] | ((uint32_t)resp[5] << 8) |
                          ((uint32_t)resp[6] << 16) | ((uint32_t)resp[7] << 24);
        }
        return HEXMEOW_SDO_ERR_ABORT;
    }
    /* Expedited upload response: bits 7-5 = 010 (SCS=2), bit 1 = e=1. */
    if ((resp[0] & 0xE0) != SDO_SRV_UL_RESPONSE_EXP) {
        return HEXMEOW_SDO_ERR_PROTOCOL;
    }
    if ((resp[0] & 0x02) == 0) {
        /* Non-expedited (segmented) -- we don't support this for the demo. */
        return HEXMEOW_SDO_ERR_PROTOCOL;
    }
    uint8_t n_unused = (resp[0] >> 2) & 0x3;
    uint8_t data_len = ((resp[0] & 0x01) ? (4 - n_unused) : 4);
    if (data_len > 4) return HEXMEOW_SDO_ERR_PROTOCOL;
    memcpy(out, &resp[4], data_len);
    *out_len = data_len;
    return HEXMEOW_SDO_OK;
}

/* ---- typed helpers ---- */

int hexmeow_sdo_dl_u8(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint8_t v) {
    return hexmeow_sdo_download(s, n, i, sub, &v, 1, HEXMEOW_SDO_TIMEOUT_MS, NULL);
}
int hexmeow_sdo_dl_u16(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint16_t v) {
    uint8_t buf[2] = {(uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF)};
    return hexmeow_sdo_download(s, n, i, sub, buf, 2, HEXMEOW_SDO_TIMEOUT_MS, NULL);
}
int hexmeow_sdo_dl_u32(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint32_t v) {
    uint8_t buf[4] = {
        (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF),
        (uint8_t)((v >> 16) & 0xFF), (uint8_t)((v >> 24) & 0xFF),
    };
    return hexmeow_sdo_download(s, n, i, sub, buf, 4, HEXMEOW_SDO_TIMEOUT_MS, NULL);
}
int hexmeow_sdo_dl_i8(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, int8_t v) {
    uint8_t b = (uint8_t)v;
    return hexmeow_sdo_download(s, n, i, sub, &b, 1, HEXMEOW_SDO_TIMEOUT_MS, NULL);
}
int hexmeow_sdo_dl_f32(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, float v) {
    uint32_t bits;
    memcpy(&bits, &v, 4);
    return hexmeow_sdo_dl_u32(s, n, i, sub, bits);
}

int hexmeow_sdo_ul_u8(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint8_t *v) {
    uint8_t buf[4] = {0};
    uint8_t got = 0;
    int rc = hexmeow_sdo_upload(s, n, i, sub, buf, &got, HEXMEOW_SDO_TIMEOUT_MS, NULL);
    if (rc) return rc;
    if (got < 1) return HEXMEOW_SDO_ERR_PROTOCOL;
    *v = buf[0];
    return 0;
}
int hexmeow_sdo_ul_u16(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint16_t *v) {
    uint8_t buf[4] = {0};
    uint8_t got = 0;
    int rc = hexmeow_sdo_upload(s, n, i, sub, buf, &got, HEXMEOW_SDO_TIMEOUT_MS, NULL);
    if (rc) return rc;
    if (got < 2) return HEXMEOW_SDO_ERR_PROTOCOL;
    *v = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return 0;
}
int hexmeow_sdo_ul_u32(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, uint32_t *v) {
    uint8_t buf[4] = {0};
    uint8_t got = 0;
    int rc = hexmeow_sdo_upload(s, n, i, sub, buf, &got, HEXMEOW_SDO_TIMEOUT_MS, NULL);
    if (rc) return rc;
    if (got < 4) return HEXMEOW_SDO_ERR_PROTOCOL;
    *v = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
         ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
    return 0;
}
int hexmeow_sdo_ul_f32(hexmeow_sdo_t *s, uint8_t n, uint16_t i, uint8_t sub, float *v) {
    uint32_t bits = 0;
    int rc = hexmeow_sdo_ul_u32(s, n, i, sub, &bits);
    if (rc) return rc;
    memcpy(v, &bits, 4);
    return 0;
}

const char *hexmeow_sdo_strerr(int rc) {
    switch (rc) {
        case HEXMEOW_SDO_OK:           return "OK";
        case HEXMEOW_SDO_ERR_IO:       return "IO error";
        case HEXMEOW_SDO_ERR_TIMEOUT:  return "timeout";
        case HEXMEOW_SDO_ERR_ABORT:    return "server abort";
        case HEXMEOW_SDO_ERR_PROTOCOL: return "protocol error";
        case HEXMEOW_SDO_ERR_ARG:      return "invalid argument";
        default:                         return "unknown";
    }
}
