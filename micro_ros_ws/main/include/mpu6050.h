#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int sda_gpio;
    int scl_gpio;
    uint32_t i2c_frequency_hz;
    uint8_t address;
} mpu6050_config_t;

typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
} mpu6050_raw_t;

esp_err_t mpu6050_init(const mpu6050_config_t *config);
esp_err_t mpu6050_read_id(uint8_t *id);
esp_err_t mpu6050_read_raw(mpu6050_raw_t *data);

#endif