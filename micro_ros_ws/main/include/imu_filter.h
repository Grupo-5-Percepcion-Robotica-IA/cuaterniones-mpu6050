#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "quaternion.h"

typedef struct {
    quaternion_t orientation;
    float alpha;
    int initialized;
} imu_filter_t;

// alpha debe estar entre 0 y 1.
int imu_filter_init(imu_filter_t *filter, float alpha);

// accel en m/s², gyro en rad/s y dt en segundos.
int imu_filter_update(imu_filter_t *filter,const float accel[3],const float gyro[3],float dt);

#endif