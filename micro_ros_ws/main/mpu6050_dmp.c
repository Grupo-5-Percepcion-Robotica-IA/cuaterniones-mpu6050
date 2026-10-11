#include "mpu6050_dmp.h"
#include "mpu6050_bus.h"
#include "dmp_firmware.h"

#include <math.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Secuencia de arranque adaptada de dmpInitialize() de la libreria
 * I2Cdev / MPU6050 de Jeff Rowberg (MIT, Copyright (c) 2021 Jeff Rowberg).
 */

// Registros
#define REG_XG_OFFS_TC    0x00
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_MOT_THR       0x1F
#define REG_MOT_DUR       0x20
#define REG_ZRMOT_THR     0x21
#define REG_ZRMOT_DUR     0x22
#define REG_I2C_SLV0_ADDR 0x25
#define REG_INT_ENABLE    0x38
#define REG_INT_STATUS    0x3A
#define REG_USER_CTRL     0x6A
#define REG_PWR_MGMT_1    0x6B
#define REG_BANK_SEL      0x6D
#define REG_MEM_START     0x6E
#define REG_MEM_R_W       0x6F
#define REG_DMP_CFG_1     0x70
#define REG_DMP_CFG_2     0x71
#define REG_FIFO_COUNT_H  0x72
#define REG_FIFO_R_W      0x74

// Bits de USER_CTRL
#define USERCTRL_DMP_EN       7
#define USERCTRL_FIFO_EN      6
#define USERCTRL_I2C_MST_EN   5
#define USERCTRL_DMP_RESET    3
#define USERCTRL_FIFO_RESET   2
#define USERCTRL_I2C_MST_RST  1

#define MEMORY_CHUNK_SIZE 16
#define FIFO_SIZE         1024

// Si se acumulan mas paquetes que estos, estamos atrasados: se vacia la FIFO.
#define MAX_PACKETS_PER_READ 4


// ---------------------------------------------------------------------
// Ayudas de bajo nivel
// ---------------------------------------------------------------------

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    return mpu6050_write_bytes(reg, &value, 1);
}

static esp_err_t read_reg(uint8_t reg, uint8_t *value)
{
    return mpu6050_read_bytes(reg, value, 1);
}

// Cambia solo los bits indicados: lee, modifica, escribe.
static esp_err_t update_bits(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current;
    esp_err_t err = read_reg(reg, &current);

    if (err != ESP_OK) {
        return err;
    }

    current = (uint8_t)((current & ~mask) | (value & mask));

    return write_reg(reg, current);
}

static esp_err_t set_bit(uint8_t reg, uint8_t bit, int on)
{
    return update_bits(reg, (uint8_t)(1u << bit), on ? (uint8_t)(1u << bit) : 0);
}

#define TRY(call)                              \
    do {                                       \
        esp_err_t _e = (call);                 \
        if (_e != ESP_OK) {                    \
            return _e;                         \
        }                                      \
    } while (0)

static esp_err_t select_memory(uint8_t bank, uint8_t address)
{
    TRY(write_reg(REG_BANK_SEL, bank & 0x1F));
    return write_reg(REG_MEM_START, address);
}

// Escribe un bloque en la memoria interna del DMP y lo verifica.
static esp_err_t write_memory_block(
    const uint8_t *data,
    uint16_t size,
    uint8_t bank,
    uint8_t address
)
{
    uint8_t check[MEMORY_CHUNK_SIZE];

    TRY(select_memory(bank, address));

    for (uint16_t i = 0; i < size;) {
        uint16_t chunk = MEMORY_CHUNK_SIZE;

        // No pasarse del final de los datos.
        if (i + chunk > size) {
            chunk = size - i;
        }

        // No pasarse del final del banco (256 bytes).
        if (chunk > (uint16_t)(256 - address)) {
            chunk = 256 - address;
        }

        TRY(mpu6050_write_bytes(REG_MEM_R_W, &data[i], chunk));

        // Verificacion: se vuelve a leer lo escrito.
        TRY(select_memory(bank, address));
        TRY(mpu6050_read_bytes(REG_MEM_R_W, check, chunk));

        for (uint16_t j = 0; j < chunk; j++) {
            if (check[j] != data[i + j]) {
                return ESP_ERR_INVALID_RESPONSE;
            }
        }

        i += chunk;
        address = (uint8_t)(address + chunk);   // da la vuelta en 256

        if (i < size) {
            if (address == 0) {
                bank++;
            }

            TRY(select_memory(bank, address));
        }
    }

    return ESP_OK;
}

static esp_err_t reset_fifo(void)
{
    return set_bit(REG_USER_CTRL, USERCTRL_FIFO_RESET, 1);
}


// ---------------------------------------------------------------------
// API publica
// ---------------------------------------------------------------------

esp_err_t mpu6050_dmp_init(void)
{
    uint8_t dummy;

    // Reset completo del sensor.
    TRY(write_reg(REG_PWR_MGMT_1, 0x80));
    vTaskDelay(pdMS_TO_TICKS(100));

    // Despertar (el reset lo deja dormido).
    TRY(set_bit(REG_PWR_MGMT_1, 6, 0));

    // Lectura de la revision de hardware (paso de la libreria original).
    TRY(write_reg(REG_BANK_SEL, 0x70));
    TRY(write_reg(REG_MEM_START, 0x06));
    TRY(read_reg(REG_MEM_R_W, &dummy));
    TRY(write_reg(REG_BANK_SEL, 0x00));

    // Dejar el I2C master interno apagado y reiniciado.
    TRY(write_reg(REG_I2C_SLV0_ADDR, 0x7F));
    TRY(set_bit(REG_USER_CTRL, USERCTRL_I2C_MST_EN, 0));
    TRY(write_reg(REG_I2C_SLV0_ADDR, 0x68));
    TRY(set_bit(REG_USER_CTRL, USERCTRL_I2C_MST_RST, 1));
    vTaskDelay(pdMS_TO_TICKS(20));

    // Reloj: PLL con el giroscopio Z.
    TRY(update_bits(REG_PWR_MGMT_1, 0x07, 0x03));

    // Interrupciones: desborde de FIFO y DMP.
    TRY(write_reg(REG_INT_ENABLE, (1 << 4) | (1 << 1)));

    // 1 kHz / (1 + 4) = 200 Hz.
    TRY(write_reg(REG_SMPLRT_DIV, 4));

    // Sincronizacion externa: TEMP_OUT_L[0].
    TRY(update_bits(REG_CONFIG, 0x38, 1 << 3));

    // Filtro digital: 42 Hz.
    TRY(update_bits(REG_CONFIG, 0x07, 0x03));

    // Giroscopio a +-2000 grados/s (lo exige el firmware).
    TRY(update_bits(REG_GYRO_CONFIG, 0x18, 0x03 << 3));

    // Cargar el firmware del DMP y verificarlo.
    TRY(write_memory_block(dmp_firmware, MPU6050_DMP_CODE_SIZE, 0, 0));

    // Divisor de la tasa de la FIFO, escrito dentro del firmware.
    const uint8_t rate_patch[2] = {0x00, MPU6050_DMP_FIFO_RATE_DIVISOR};
    TRY(write_memory_block(rate_patch, 2, 0x02, 0x16));

    // Direccion de arranque del programa del DMP (0x0300).
    TRY(write_reg(REG_DMP_CFG_1, 0x03));
    TRY(write_reg(REG_DMP_CFG_2, 0x00));

    // Borrar el flag OTP.
    TRY(set_bit(REG_XG_OFFS_TC, 0, 0));

    // Umbrales de deteccion de movimiento (valores de la libreria original).
    TRY(write_reg(REG_MOT_THR, 2));
    TRY(write_reg(REG_ZRMOT_THR, 156));
    TRY(write_reg(REG_MOT_DUR, 80));
    TRY(write_reg(REG_ZRMOT_DUR, 0));

    // Habilitar la FIFO y reiniciar el DMP.
    TRY(set_bit(REG_USER_CTRL, USERCTRL_FIFO_EN, 1));
    TRY(set_bit(REG_USER_CTRL, USERCTRL_DMP_RESET, 1));

    // Queda apagado hasta mpu6050_dmp_start().
    TRY(set_bit(REG_USER_CTRL, USERCTRL_DMP_EN, 0));

    TRY(reset_fifo());
    TRY(read_reg(REG_INT_STATUS, &dummy));

    return ESP_OK;
}

esp_err_t mpu6050_dmp_start(void)
{
    uint8_t dummy;

    TRY(set_bit(REG_USER_CTRL, USERCTRL_DMP_EN, 1));
    TRY(reset_fifo());
    TRY(read_reg(REG_INT_STATUS, &dummy));

    return ESP_OK;
}

static int32_t be32(const uint8_t *p)
{
    return (int32_t)(
        ((uint32_t)p[0] << 24) |
        ((uint32_t)p[1] << 16) |
        ((uint32_t)p[2] << 8)  |
        (uint32_t)p[3]
    );
}

// Los 16 bits de arriba de un campo de 32 bits.
static int16_t be16_top(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

// Convierte un paquete de 42 bytes. Devuelve 0 si es valido.
static int parse_packet(const uint8_t *p, mpu6050_dmp_packet_t *out)
{
    // Cuaternion en Q30.
    const float k = 1.0f / 1073741824.0f;

    quaternion_t q = {
        .w = (float)be32(&p[0])  * k,
        .x = (float)be32(&p[4])  * k,
        .y = (float)be32(&p[8])  * k,
        .z = (float)be32(&p[12]) * k,
    };

    // Un paquete bien alineado tiene un cuaternion de norma ~1.
    float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

    if (!isfinite(norm) || norm < 0.9f || norm > 1.1f) {
        return -1;
    }

    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;

    out->q = q;

    out->gyro[0] = be16_top(&p[16]);
    out->gyro[1] = be16_top(&p[20]);
    out->gyro[2] = be16_top(&p[24]);

    out->accel[0] = be16_top(&p[28]);
    out->accel[1] = be16_top(&p[32]);
    out->accel[2] = be16_top(&p[36]);

    out->valid = 1;

    return 0;
}

esp_err_t mpu6050_dmp_read_latest(mpu6050_dmp_packet_t *latest, uint8_t *consumed)
{
    if (latest == NULL || consumed == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *consumed = 0;
    latest->valid = 0;

    uint8_t count_bytes[2];

    TRY(mpu6050_read_bytes(REG_FIFO_COUNT_H, count_bytes, 2));

    uint16_t count = ((uint16_t)count_bytes[0] << 8) | count_bytes[1];

    // Desborde: los paquetes pueden haber quedado desalineados.
    if (count >= FIFO_SIZE) {
        TRY(reset_fifo());
        return ESP_OK;
    }

    uint16_t packets = count / MPU6050_DMP_PACKET_SIZE;

    if (packets == 0) {
        return ESP_OK;
    }

    // Demasiado atrasados: es mejor empezar de cero que leer datos viejos.
    if (packets > MAX_PACKETS_PER_READ) {
        TRY(reset_fifo());
        return ESP_OK;
    }

    uint8_t buffer[MAX_PACKETS_PER_READ * MPU6050_DMP_PACKET_SIZE];
    size_t bytes = (size_t)packets * MPU6050_DMP_PACKET_SIZE;

    TRY(mpu6050_read_bytes(REG_FIFO_R_W, buffer, bytes));

    // Solo interesa el ultimo paquete.
    const uint8_t *last = &buffer[bytes - MPU6050_DMP_PACKET_SIZE];

    if (parse_packet(last, latest) != 0) {
        TRY(reset_fifo());
        return ESP_ERR_INVALID_RESPONSE;
    }

    *consumed = (uint8_t)packets;

    return ESP_OK;
}