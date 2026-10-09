#include "imu_motion.h"

#include <math.h>
#include <stddef.h>

void imu_motion_reset(imu_motion_t *motion)
{
    if (motion == NULL) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        motion->velocity[i] = 0.0f;
        motion->position[i] = 0.0f;
    }
}

int imu_motion_update(
    imu_motion_t *motion,
    const float acceleration[3],
    float dt
)
{
    if (motion == NULL || acceleration == NULL ||
        !isfinite(dt) || dt <= 0.0f) {
        return -1;
    }

    // Validar todos los ejes antes de modificar el estado.
    for (int i = 0; i < 3; i++) {
        if (!isfinite(acceleration[i])) {
            return -1;
        }
    }

    for (int i = 0; i < 3; i++) {
        motion->position[i] +=
            motion->velocity[i] * dt +
            0.5f * acceleration[i] * dt * dt;

        motion->velocity[i] += acceleration[i] * dt;
    }

    return 0;
}