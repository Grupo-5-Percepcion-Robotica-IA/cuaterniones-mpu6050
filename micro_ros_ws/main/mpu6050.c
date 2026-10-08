#include "mpu6050.h"
#include "driver/i2c_master.h"

#define MPU6050_REG_WHO_AM_I 0x75
#define I2C_TIMEOUT_MS      100

// Privados del módulo: main.c no necesita conocerlos.
static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t device_handle = NULL;

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
    }

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
    (void)data;

    // Lo implementaremos después.
    return ESP_ERR_NOT_SUPPORTED;
}