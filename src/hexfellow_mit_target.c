#include "hexfellow_mit_target.h"

#include <string.h>

hexfellow_mit_mapping_t hexfellow_mit_mapping_default(void) {
    hexfellow_mit_mapping_t m = {
        .position_min = -0.5f,  .position_max = 0.5f,
        .velocity_min = -10.0f, .velocity_max = 10.0f,
        .torque_min   = -10.0f, .torque_max   = 10.0f,
        .kp_min       = 0.0f,   .kp_max       = 100.0f,
        .kd_min       = 0.0f,   .kd_max       = 20.0f,
    };
    return m;
}

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline uint32_t float_to_uint(float x, float xmin, float xmax,
                                     uint32_t bits) {
    float span = xmax - xmin;
    float scale = (float)((1u << bits) - 1u);
    return (uint32_t)(((x - xmin) * scale) / span);
}

static inline void store_u32_le(uint8_t dst[4], uint32_t v) {
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
    dst[2] = (uint8_t)((v >> 16) & 0xFFu);
    dst[3] = (uint8_t)((v >> 24) & 0xFFu);
}

void hexfellow_mit_target_pack(const hexfellow_mit_target_t *t,
                               const hexfellow_mit_mapping_t *m,
                               uint8_t out[8]) {
    float position = clampf(t->position, m->position_min, m->position_max);
    float velocity = clampf(t->velocity, m->velocity_min, m->velocity_max);
    float torque   = clampf(t->torque,   m->torque_min,   m->torque_max);
    float kp       = clampf(t->kp,       m->kp_min,       m->kp_max);
    float kd       = clampf(t->kd,       m->kd_min,       m->kd_max);

    uint32_t pos_u  = float_to_uint(position, m->position_min, m->position_max, 16);
    uint32_t vel_u  = float_to_uint(velocity, m->velocity_min, m->velocity_max, 12);
    uint32_t torq_u = float_to_uint(torque,   m->torque_min,   m->torque_max,   12);
    uint32_t kp_u   = float_to_uint(kp,       m->kp_min,       m->kp_max,       12);
    uint32_t kd_u   = float_to_uint(kd,       m->kd_min,       m->kd_max,       12);

    uint32_t lower_u32 = torq_u | (kd_u << 12) | ((kp_u & 0xFFu) << 24);
    uint32_t upper_u32 = (kp_u >> 8) | (vel_u << 4) | (pos_u << 16);

    store_u32_le(out,     lower_u32);
    store_u32_le(out + 4, upper_u32);
}
