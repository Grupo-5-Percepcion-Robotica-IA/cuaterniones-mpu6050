#include "imu_filter.h"

#include <math.h>
#include <stddef.h>

// Construye una orientación cuya vertical coincide
// con la dirección medida por el acelerómetro.
static int orientation_from_accel(
    quaternion_t prediction,
    const float accel[3],
    quaternion_t *result
)
{
    float norm = sqrtf(
        accel[0] * accel[0] +
        accel[1] * accel[1] +
        accel[2] * accel[2]
    );

    if (!isfinite(norm) || norm < 1e-6f) {
        return -1;
    }

    // Copia normalizada: accel original conserva su magnitud.
    quaternion_t direction = {
        .w = 0.0f,
        .x = accel[0] / norm,
        .y = accel[1] / norm,
        .z = accel[2] / norm,
    };

    // Dirección del acelerómetro expresada en el mundo.
    quaternion_t world_direction = quaternion_multiply(
        quaternion_multiply(prediction, direction),
        quaternion_conjugate(prediction)
    );

    // Rotación mínima desde world_direction hasta [0, 0, 1].
    // Producto cruz: v × Z = [vy, -vx, 0].
    quaternion_t correction = {
        .w = 1.0f + world_direction.z,
        .x = world_direction.y,
        .y = -world_direction.x,
        .z = 0.0f,
    };

    // Caso especial: direcciones prácticamente opuestas.
    if (correction.w < 1e-6f) {
        correction = (quaternion_t) {
            .w = 0.0f,
            .x = 1.0f,
            .y = 0.0f,
            .z = 0.0f,
        };
    }

    if (quaternion_normalize(&correction) != 0) {
        return -1;
    }

    // Corrección expresada en el mundo: multiplica a izquierda.
    *result = quaternion_multiply(correction, prediction);

    return quaternion_normalize(result);
}

int imu_filter_init(imu_filter_t *filter, float alpha)
{
    if (filter == NULL ||
        !isfinite(alpha) || alpha < 0.0f || alpha > 1.0f) {
        return -1;
    }

    filter->orientation = (quaternion_t) {
        .w = 1.0f,
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
    };

    filter->alpha = alpha;
    filter->initialized = 0;

    return 0;
}

int imu_filter_update(
    imu_filter_t *filter,
    const float accel[3],
    const float gyro[3],
    float dt
)
{
    if (filter == NULL || accel == NULL || gyro == NULL ||
        !isfinite(dt) || dt <= 0.0f) {
        return -1;
    }

    for (int i = 0; i < 3; i++) {
        if (!isfinite(accel[i]) || !isfinite(gyro[i])) {
            return -1;
        }
    }

    // Primera muestra: inicializar inclinación con acelerómetro.
    if (!filter->initialized) {
        quaternion_t initial;

        if (orientation_from_accel(
                filter->orientation, accel, &initial) != 0) {
            return -1;
        }

        filter->orientation = initial;
        filter->initialized = 1;

        return 0;
    }

    // Predicción desde la orientación fusionada anterior.
    quaternion_t q_gyro = filter->orientation;

    if (quaternion_integrate_gyro(&q_gyro, gyro, dt) != 0) {
        return -1;
    }

    quaternion_t q_acc;

    // Si no hay dirección válida, conservar solo la predicción.
    if (orientation_from_accel(q_gyro, accel, &q_acc) != 0) {
        filter->orientation = q_gyro;
        return 0;
    }

    // q y -q representan la misma orientación.
    float dot = q_gyro.w * q_acc.w +
                q_gyro.x * q_acc.x +
                q_gyro.y * q_acc.y +
                q_gyro.z * q_acc.z;

    if (dot < 0.0f) {
        q_acc.w = -q_acc.w;
        q_acc.x = -q_acc.x;
        q_acc.y = -q_acc.y;
        q_acc.z = -q_acc.z;
    }

    float alpha = filter->alpha;
    float beta = 1.0f - alpha;

    quaternion_t fused = {
        .w = alpha * q_gyro.w + beta * q_acc.w,
        .x = alpha * q_gyro.x + beta * q_acc.x,
        .y = alpha * q_gyro.y + beta * q_acc.y,
        .z = alpha * q_gyro.z + beta * q_acc.z,
    };

    if (quaternion_normalize(&fused) != 0) {
        return -1;
    }

    filter->orientation = fused;

    return 0;
}