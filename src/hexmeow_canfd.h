/*
 * Raw SocketCAN-FD helpers for hexmeow demos.
 *
 * These wrappers open a SOCK_RAW socket with CAN_RAW_FD_FRAMES enabled, so a
 * single fd can transmit and receive both classic 8-byte CAN frames and
 * 64-byte CAN-FD frames.
 *
 * Error policy: on open failure we abort the whole program (this is a demo,
 * not a library). Send/recv functions return non-zero on error and do not
 * print messages themselves -- the caller decides whether to abort.
 */
#ifndef HEXMEOW_CANFD_H
#define HEXMEOW_CANFD_H

#include <stdint.h>
#include <stddef.h>
#include <linux/can.h>

/* Open `ifname` (e.g. "can0") and return a SocketCAN fd configured for
 * CAN-FD transmission/reception. Aborts on any failure. */
int hexmeow_canfd_open(const char *ifname);

/* Send a classic CAN frame (DLC <= 8). Returns 0 on success, -1 on error. */
int hexmeow_canfd_send_classic(int fd, uint16_t cob_id,
                                 const void *data, uint8_t len);

/* Send a CAN-FD frame. `len` may be any value 0..64; the function will pad
 * up to the nearest legal FD DLC (0..8, 12, 16, 20, 24, 32, 48, 64). The
 * BRS (Bit Rate Switch) flag is always set -- some CAN-FD controllers
 * will not accept FD frames whose data phase is at the arbitration
 * bitrate, even though it is nominally legal. Returns 0 on success, -1
 * on error. */
int hexmeow_canfd_send_fd(int fd, uint16_t cob_id,
                            const void *data, uint8_t len);

/* Non-blocking receive (MSG_DONTWAIT). Returns immediately whether or
 * not a frame is ready. Sets *is_fd = 1 for an FD frame, 0 for classic.
 *
 * Returns:  0 on success
 *          -1 on error (errno set)
 *          -2 if no frame was waiting (EAGAIN/EWOULDBLOCK) */
int hexmeow_canfd_recv(int fd, struct canfd_frame *frame, int *is_fd);

/* Round `len` up to the nearest legal CAN-FD payload length. */
uint8_t hexmeow_canfd_pad_len(uint8_t len);

#endif /* HEXMEOW_CANFD_H */
