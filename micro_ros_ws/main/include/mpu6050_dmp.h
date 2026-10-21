#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "quaternion.h"

/*
 * Driver del DMP del MPU-6050.
 *
 * Basado en la secuencia de arranque de la libreria I2Cdev / MPU6050 de
 * Jeff Rowberg (MIT, Copyright (c) 2021 Jeff Rowberg), adaptado al driver
 * I2C nuevo de ESP-IDF (el mismo bus que ya abre mpu6050_init()).
 *
 * Uso:
 *   mpu6050_init(...)          // como siempre
 *   mpu6050_dmp_init()         // resetea el sensor, carga el firmware, DMP apagado
 *   ... calibrar giroscopio ...
 *   mpu6050_dmp_start()        // enciende el DMP
 *   mpu6050_dmp_read_latest()  // cada vez que quieras el ultimo paquete
 *
 * ATENCION: mpu6050_dmp_init() deja el giroscopio en +-2000 grados/s
 * (lo exige el firmware). Eso cambia la escala de las lecturas crudas
 * del giroscopio: 16.4 LSB por grado/s (en vez de 131).
 */

// Tamano del paquete que escribe el DMP en la FIFO.
#define MPU6050_DMP_PACKET_SIZE 42

// El DMP trabaja a 200 Hz internos. Divisor 1 -> un paquete cada 10 ms (100 Hz).
// Con 0 seria uno cada 5 ms (200 Hz), pero ocupa mas el bus I2C.
#ifndef MPU6050_DMP_FIFO_RATE_DIVISOR
#define MPU6050_DMP_FIFO_RATE_DIVISOR 1
#endif

// Tiempo entre paquetes, en segundos.
#define MPU6050_DMP_PERIOD_S ((1.0f + (float)MPU6050_DMP_FIFO_RATE_DIVISOR) / 200.0f)

// Sensibilidad del giroscopio con +-2000 grados/s.
#define MPU6050_DMP_GYRO_LSB_PER_DPS 16.4f

// Aceleracion dentro del paquete: 8192 LSB por g (la mitad que el
// registro crudo con +-2 g, que da 16384 LSB por g).
#define MPU6050_DMP_ACCEL_LSB_PER_G 8192.0f

typedef struct {
    quaternion_t q;     // orientacion sensor -> mundo (Z arriba), ya normalizada
    int16_t accel[3];   // cuentas, 8192 LSB/g
    int16_t gyro[3];    // cuentas, 16.4 LSB por grado/s
    uint8_t valid;      // 1 si el paquete es utilizable
} mpu6050_dmp_packet_t;

// Resetea el sensor, configura 200 Hz / DLPF 42 Hz / +-2000 dps,
// carga y verifica el firmware del DMP. Deja el DMP apagado.
esp_err_t mpu6050_dmp_init(void);

// Enciende el DMP y vacia la FIFO.
esp_err_t mpu6050_dmp_start(void);

// Lee de la FIFO el paquete MAS NUEVO.
//   *consumed = cuantos paquetes se sacaron de la FIFO (0 si no habia ninguno).
//   Devuelve ESP_OK tambien cuando todavia no hay paquete (consumed = 0).
//   Si la FIFO se desbordo, la vacia y devuelve consumed = 0.
esp_err_t mpu6050_dmp_read_latest(mpu6050_dmp_packet_t *latest, uint8_t *consumed);