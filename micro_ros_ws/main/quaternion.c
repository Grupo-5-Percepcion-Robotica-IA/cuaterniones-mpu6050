#include "quaternion.h"

#include <math.h>
#include <stddef.h>

quaternion_t quaternion_multiply(
    quaternion_t a,
    quaternion_t b
)
{
    quaternion_t result = {
        .w = a.w * b.w - a.x * b.x
                         - a.y * b.y - a.z * b.z,

        .x = a.w * b.x + a.x * b.w
                         + a.y * b.z - a.z * b.y,

        .y = a.w * b.y - a.x * b.z
                         + a.y * b.w + a.z * b.x,

        .z = a.w * b.z + a.x * b.y
                         - a.y * b.x + a.z * b.w,
    };

    return result;
}

quaternion_t quaternion_conjugate(quaternion_t q)
{
    quaternion_t result = {
        .w = q.w,
        .x = -q.x,
        .y = -q.y,
        .z = -q.z,
    };

    return result;
}

int quaternion_normalize(quaternion_t *q)
{
    if (q == NULL) {
        return -1;
    }

    float norm = sqrtf(
        q->w * q->w +
        q->x * q->x +
        q->y * q->y +
        q->z * q->z
    );

    if (!isfinite(norm) || norm < 1e-8f) {
        return -1;
    }

    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;

    return 0;
}


int quaternion_integrate_gyro(quaternion_t *q, const float gyro[3], float dt)
{
    if (q == NULL || gyro == NULL ||
        !isfinite(dt) || dt <= 0.0f) {
        return -1;
    }

    quaternion_t omega = {
        .w = 0.0f,
        .x = gyro[0],
        .y = gyro[1],
        .z = gyro[2],
    };

    quaternion_t derivative = quaternion_multiply(*q, omega);

    quaternion_t next = {
        .w = q->w + 0.5f * derivative.w * dt,
        .x = q->x + 0.5f * derivative.x * dt,
        .y = q->y + 0.5f * derivative.y * dt,
        .z = q->z + 0.5f * derivative.z * dt,
    };

    if (quaternion_normalize(&next) != 0) {
        return -1;
    }

    *q = next;
    return 0;
}