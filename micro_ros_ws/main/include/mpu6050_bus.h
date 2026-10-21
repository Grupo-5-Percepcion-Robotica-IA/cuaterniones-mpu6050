#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Lectura/escritura de varios bytes seguidos. Estan en mpu6050.c y usan el
// mismo bus I2C que mpu6050_init(). Las usa el driver del DMP.
esp_err_t mpu6050_read_bytes(uint8_t reg, uint8_t *data, size_t len);
esp_err_t mpu6050_write_bytes(uint8_t reg, const uint8_t *data, size_t len);