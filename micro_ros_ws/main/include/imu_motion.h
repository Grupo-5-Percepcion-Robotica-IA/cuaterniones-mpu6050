#ifndef IMU_MOTION_H
#define IMU_MOTION_H

typedef struct {
    float velocity[3];  // m/s
    float position[3];  // m
} imu_motion_t;

void imu_motion_reset(imu_motion_t *motion);

int imu_motion_update(imu_motion_t *motion, const float acceleration[3],float dt);

#endif