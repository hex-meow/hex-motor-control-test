#include "hexfellow_canfd.h"

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

static const uint8_t kFdLengths[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64,
};

uint8_t hexfellow_canfd_pad_len(uint8_t len) {
    for (size_t i = 0; i < sizeof(kFdLengths); i++) {
        if (kFdLengths[i] >= len) {
            return kFdLengths[i];
        }
    }
    return 64;
}

int hexfellow_canfd_open(const char *ifname) {
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        fprintf(stderr, "[hexfellow_canfd] socket(PF_CAN) failed: %s\n",
                strerror(errno));
        exit(EXIT_FAILURE);
    }

    int enable_fd = 1;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd,
                   sizeof(enable_fd)) < 0) {
        fprintf(stderr,
                "[hexfellow_canfd] setsockopt(CAN_RAW_FD_FRAMES) on '%s' "
                "failed: %s. Make sure the interface is UP with `fd on`.\n",
                ifname, strerror(errno));
        close(fd);
        exit(EXIT_FAILURE);
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr,
                "[hexfellow_canfd] ioctl(SIOCGIFINDEX) for '%s' failed: %s\n",
                ifname, strerror(errno));
        close(fd);
        exit(EXIT_FAILURE);
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "[hexfellow_canfd] bind '%s' failed: %s\n", ifname,
                strerror(errno));
        close(fd);
        exit(EXIT_FAILURE);
    }
    return fd;
}

int hexfellow_canfd_send_classic(int fd, uint16_t cob_id, const void *data,
                                 uint8_t len) {
    if (len > 8) {
        errno = EINVAL;
        return -1;
    }
    struct can_frame f;
    memset(&f, 0, sizeof(f));
    f.can_id = cob_id & CAN_SFF_MASK;
    f.can_dlc = len;
    if (data && len > 0) memcpy(f.data, data, len);
    ssize_t n = write(fd, &f, sizeof(f));
    return (n == (ssize_t)sizeof(f)) ? 0 : -1;
}

int hexfellow_canfd_send_fd(int fd, uint16_t cob_id, const void *data,
                            uint8_t len) {
    if (len > 64) {
        errno = EINVAL;
        return -1;
    }
    struct canfd_frame f;
    memset(&f, 0, sizeof(f));
    f.can_id = cob_id & CAN_SFF_MASK;
    uint8_t plen = hexfellow_canfd_pad_len(len);
    f.len = plen;
    /* Always request the bit-rate switch on CAN-FD frames, matching the
     * production Rust code (`socketcan::id::FdFlags::BRS`). Some CAN-FD
     * controllers will refuse to actually transmit an FD frame whose data
     * phase is at the arbitration bitrate, which silently jams the kernel
     * TX queue and blocks subsequent writes -- including heartbeats. */
    f.flags = CANFD_BRS;
    if (data && len > 0) memcpy(f.data, data, len);
    ssize_t n = write(fd, &f, sizeof(f));
    return (n == (ssize_t)sizeof(f)) ? 0 : -1;
}

int hexfellow_canfd_recv(int fd, struct canfd_frame *frame, int *is_fd) {
    /* Non-blocking receive. MSG_DONTWAIT makes recv() return immediately
     * with EAGAIN if no frame is available. This is critical for the
     * control thread's drain loop: motors send TPDO1 at 1 ms cadence, so
     * a blocking recv() with SO_RCVTIMEO=1ms can be "ambushed" forever
     * (each timeout coincides with the next frame arriving), starving the
     * RPDO send path. CAN_RAW_FD_FRAMES sockets deliver classic frames as
     * 16 bytes (sizeof struct can_frame) and FD frames as 72 bytes
     * (sizeof struct canfd_frame) -- we read into the larger buffer
     * either way. */
    ssize_t n = recv(fd, frame, sizeof(*frame), MSG_DONTWAIT);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return -2;
        return -1;
    }
    if (n == (ssize_t)sizeof(struct canfd_frame)) {
        if (is_fd) *is_fd = 1;
    } else if (n == (ssize_t)sizeof(struct can_frame)) {
        if (is_fd) *is_fd = 0;
    } else {
        errno = EBADMSG;
        return -1;
    }
    return 0;
}
