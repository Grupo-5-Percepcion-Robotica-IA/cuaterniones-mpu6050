#include "mpu6050.h"
#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MPU6050_REG_WHO_AM_I 0x75
#define I2C_TIMEOUT_MS      100

#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_PWR_MGMT_2   0x6C
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B


// Privados del módulo: main.c no necesita conocerlos.
static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t device_handle = NULL;


static esp_err_t mpu6050_write_register(uint8_t reg, uint8_t value)
{
    if (device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buffer[2] = {
        reg,
        value
    };

    return i2c_master_transmit(
        device_handle,
        buffer,
        sizeof(buffer),
        I2C_TIMEOUT_MS
    );
}

esp_err_t mpu6050_init(const mpu6050_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bus_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = config->sda_gpio,
        .scl_io_num = config->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(
        &bus_config,
        &bus_handle
    );

    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->address,
        .scl_speed_hz = config->i2c_frequency_hz,
    };

    err = i2c_master_bus_add_device(
        bus_handle,
        &device_config,
        &device_handle
    );

    if (err != ESP_OK) {
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
        return err;
    }

    // Despertar el sensor y elegir el reloj del girómetro X.
    err = mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) {
        goto cleanup;
    }

    // Dar tiempo al sensor para estabilizarse.
    vTaskDelay(pdMS_TO_TICKS(100));

    // Habilitar los tres ejes del acelerómetro y girómetro.
    err = mpu6050_write_register(MPU6050_REG_PWR_MGMT_2, 0x00);
    if (err != ESP_OK) {
        goto cleanup;
    }

    // Filtro digital interno: DLPF_CFG = 3.
    err = mpu6050_write_register(MPU6050_REG_CONFIG, 0x03);
    if (err != ESP_OK) {
        goto cleanup;
    }

    // Frecuencia de salida: 1000 / (1 + 4) = 200 Hz.
    err = mpu6050_write_register(MPU6050_REG_SMPLRT_DIV, 0x04);
    if (err != ESP_OK) {
        goto cleanup;
    }

    // Girómetro: rango ±250 grados/s.
    err = mpu6050_write_register(MPU6050_REG_GYRO_CONFIG, 0x00);
    if (err != ESP_OK) {
        goto cleanup;
    }

    // Acelerómetro: rango ±2 g.
    err = mpu6050_write_register(MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (err != ESP_OK) {
        goto cleanup;
    }
    
    return ESP_OK;

cleanup:
    i2c_master_bus_rm_device(device_handle);
    device_handle = NULL;

    i2c_del_master_bus(bus_handle);
    bus_handle = NULL;

    return err;
}

esp_err_t mpu6050_read_id(uint8_t *id)
{
    if (id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t reg = MPU6050_REG_WHO_AM_I;

    return i2c_master_transmit_receive(
        device_handle,
        &reg,       // Registro que queremos leer.
        1,          // Enviamos un byte.
        id,         // Destino de la lectura.
        1,          // Recibimos un byte.
        I2C_TIMEOUT_MS
    );
}

esp_err_t mpu6050_read_raw(mpu6050_raw_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t buffer[14]; //14 bytes

    esp_err_t err = i2c_master_transmit_receive(
        device_handle,
        &reg,
        1,
        buffer,
        sizeof(buffer),
        I2C_TIMEOUT_MS
    );

    if (err != ESP_OK) {
        return err;
    }

    for (int i = 0; i < 3; i++) {
        int accel_index = 2 * i;
        int gyro_index = 8 + 2 * i;

        data->accel[i] = (int16_t)(
            ((uint16_t)buffer[accel_index] << 8) |
            buffer[accel_index + 1]
        );

        data->gyro[i] = (int16_t)(
            ((uint16_t)buffer[gyro_index] << 8) |
            buffer[gyro_index + 1]
        );
    }

    return ESP_OK;
}

esp_err_t mpu6050_read_ranges(uint8_t *accel_config, uint8_t *gyro_config)
{
    if (accel_config == NULL || gyro_config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // GYRO_CONFIG y ACCEL_CONFIG son consecutivos.
    uint8_t reg = MPU6050_REG_GYRO_CONFIG;
    uint8_t buffer[2];

    esp_err_t err = i2c_master_transmit_receive(
        device_handle,
        &reg,
        1,
        buffer,
        sizeof(buffer),
        I2C_TIMEOUT_MS
    );

    if (err != ESP_OK) {
        return err;
    }

    *gyro_config = buffer[0];
    *accel_config = buffer[1];

    return ESP_OK;
}

esp_err_t mpu6050_calibrate_gyro(uint32_t samples,float bias[3])
{
    if (samples == 0 || bias == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t sum[3] = {0, 0, 0};

    for (uint32_t i = 0; i < samples; i++) {
        mpu6050_raw_t raw;

        esp_err_t err = mpu6050_read_raw(&raw);

        if (err != ESP_OK) {
            return err;
        }

        for (int axis = 0; axis < 3; axis++) {
            sum[axis] += raw.gyro[axis];
        }

        // Lectura lenta para calibración, no para el filtro.
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    for (int axis = 0; axis < 3; axis++) {
        bias[axis] = (float)sum[axis] / (float)samples;
    }

    return ESP_OK;
}

esp_err_t mpu6050_average_accel(uint32_t samples, float average[3])
{
    if (samples == 0 || average == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t sum[3] = {0, 0, 0};

    for (uint32_t i = 0; i < samples; i++) {
        mpu6050_raw_t raw;

        esp_err_t err = mpu6050_read_raw(&raw);

        if (err != ESP_OK) {
            return err;
        }

        for (int axis = 0; axis < 3; axis++) {
            sum[axis] += raw.accel[axis];
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    for (int axis = 0; axis < 3; axis++) {
        average[axis] = (float)sum[axis] / (float)samples;
    }

    return ESP_OK;
}