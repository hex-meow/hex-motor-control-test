/*
 * MIT-mode target packing for hexfellow motors.
 *
 *   - position: 16 bits, range [position_min, position_max] in Rev
 *   - velocity: 12 bits, range [velocity_min, velocity_max] in Rev/s
 *   - kp:       12 bits, range [kp_min, kp_max] in Nm/Rev
 *   - kd:       12 bits, range [kd_min, kd_max] in Nm*s/Rev
 *   - torque:   12 bits, range [torque_min, torque_max] in Nm
 *
 * The 8-byte packed representation goes into the RPDO frame and is
 * unpacked by the motor firmware according to object dictionary 2004h.
 */
#ifndef HEXFELLOW_MIT_TARGET_H
#define HEXFELLOW_MIT_TARGET_H

#include <stdint.h>

typedef struct {
    float position_min, position_max;     /* Rev */
    float velocity_min, velocity_max;     /* Rev/s */
    float torque_min, torque_max;         /* Nm */
    float kp_min, kp_max;                 /* Nm/Rev */
    float kd_min, kd_max;                 /* Nm*s/Rev */
} hexfellow_mit_mapping_t;

typedef struct {
    float position;     /* Rev */
    float velocity;     /* Rev/s */
    float torque;       /* Nm (feed-forward) */
    float kp;           /* Nm/Rev */
    float kd;           /* Nm*s/Rev */
} hexfellow_mit_target_t;

/* Default mapping suitable for the demos. */
hexfellow_mit_mapping_t hexfellow_mit_mapping_default(void);

/* Pack a target into 8 little-endian bytes ready to be placed inside the
 * shared RPDO. */
void hexfellow_mit_target_pack(const hexfellow_mit_target_t *t,
                               const hexfellow_mit_mapping_t *m,
                               uint8_t out[8]);

#endif /* HEXFELLOW_MIT_TARGET_H */
