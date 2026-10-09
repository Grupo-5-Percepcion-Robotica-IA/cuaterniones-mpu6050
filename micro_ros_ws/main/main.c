#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "esp_err.h"
#include "mpu6050.h"
#include "esp_timer.h"
#include "quaternion.h"

#include <rmw_microros/rmw_microros.h>


// ===============================
// Configuracion general
// ===============================

#define ROS_DOMAIN_ID     10

#define SDA               21
#define SCL               22
#define I2C_FRECUENCY_HZ  100000
#define I2C_ADDRESS       0x68



#define NODE_NAME "esp32_node"
#define NODE_NAMESPACE ""

#define TASK_STACK_SIZE 20000
#define TASK_PRIORITY 5

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};


// ===============================
// Manejo de errores
// ===============================

#define RCCHECK(fn)                                              \
{                                                                \
    rcl_ret_t rc = fn;                                           \
    if (rc != RCL_RET_OK) {                                      \
        printf("[FALLO] linea %d: rc=%d\n", __LINE__, (int)rc);  \
        vTaskDelete(NULL);                                       \
    }                                                            \
}

#define RCSOFTCHECK(fn)                                          \
{                                                                \
    rcl_ret_t rc = fn;                                           \
    if (rc != RCL_RET_OK) {                                      \
        printf("[WARN] linea %d: rc=%d\n", __LINE__, (int)rc);   \
    }                                                            \
}

static const float accel_bias[3] = {
    480.04f,
    40.735f,
    -1581.94f
};

static const float accel_sensitivity[3] = {
    16158.85f,
    16372.115f,
    16616.44f
};


// ===============================
// Tarea principal la IMU
// ===============================

static void imu_task(void *arg)
{
    (void)arg;

    const TickType_t period = pdMS_TO_TICKS(5);
    configASSERT(period > 0);

    TickType_t last_wake = xTaskGetTickCount();
    int64_t previous_time = 0;

    uint32_t intervals = 0;
    uint32_t errors = 0;
    int64_t total_dt = 0;
    int64_t min_dt = INT64_MAX;
    int64_t max_dt = 0;

    quaternion_t orientation = {
        .w = 1.0f,
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f
    };

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        int64_t now = esp_timer_get_time();
        mpu6050_raw_t raw;

        esp_err_t err = mpu6050_read_raw(&raw);

        if (err != ESP_OK) {
            errors++;
        }

        // La primera lectura no tiene intervalo anterior.
        if (previous_time == 0) {
            previous_time = now;
            continue;
        }

        int64_t dt_us = now - previous_time;
        previous_time = now;


        total_dt += dt_us;

        if (dt_us < min_dt) {
            min_dt = dt_us;
        }

        if (dt_us > max_dt) {
            max_dt = dt_us;
        }

        float accel[3];
        float gyro[3];

        if (err == ESP_OK) {
            for (int i = 0; i < 3; i++) {
                accel[i] = (raw.accel[i] - accel_bias[i])
                        * (9.80665f / accel_sensitivity[i]);

                gyro[i] = (raw.gyro[i] - gyro_bias[i])
                        * (0.01745329252f / 131.0f);
            }

            float dt = (float)dt_us * 1e-6f;

            if (quaternion_integrate_gyro(
                    &orientation, gyro, dt) != 0) {
                printf("Error actualizando el cuaternion.\n");
            }
        }

        intervals++;

        if (intervals >= 200) {
            printf(
                "IMU: dt promedio=%.1f us | "
                "min=%lld | max=%lld | errores=%lu\n",
                (double)total_dt / intervals,
                (long long)min_dt,
                (long long)max_dt,
                (unsigned long)errors
            );

            if (err == ESP_OK) {

                for (int i = 0; i < 3; i++) {
                    accel[i] = (raw.accel[i] - accel_bias[i])
                             * (9.80665f / accel_sensitivity[i]);

                    // Cuentas -> grados/s -> radianes/s.
                    gyro[i] = (raw.gyro[i] - gyro_bias[i])
                            * (0.01745329252f / 131.0f);
                }

                printf(
                    "ACC [m/s²]: [%.2f, %.2f, %.2f] | "
                    "GYRO [rad/s]: [%.4f, %.4f, %.4f]\n",
                    accel[0], accel[1], accel[2],
                    gyro[0], gyro[1], gyro[2]
                );

                printf(
                    "q gyro: [%.4f, %.4f, %.4f, %.4f]\n",
                    orientation.w,
                    orientation.x,
                    orientation.y,
                    orientation.z
                );
            }

            intervals = 0;
            errors = 0;
            total_dt = 0;
            min_dt = INT64_MAX;
            max_dt = 0;
        }
    }
}


// ===============================
// Tarea principal de micro-ROS
// ===============================

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rclc_support_t support;

    rcl_init_options_t init_options =
        rcl_get_zero_initialized_init_options();


    // Inicializar opciones de ROS
    RCCHECK(
        rcl_init_options_init(
            &init_options,
            allocator
        )
    );


    // Configurar ROS_DOMAIN_ID
    RCCHECK(
        rcl_init_options_set_domain_id(
            &init_options,
            ROS_DOMAIN_ID
        )
    );


    // Obtener opciones de transporte de micro-ROS
    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);


    // Configurar conexion UDP con el micro-ROS Agent
    RCCHECK(
        rmw_uros_options_set_udp_address(
            CONFIG_MICRO_ROS_AGENT_IP,
            CONFIG_MICRO_ROS_AGENT_PORT,
            rmw_options
        )
    );


    // Esperar hasta que el Agent este disponible
    printf(
        "Buscando micro-ROS Agent en %s:%s...\n",
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT
    );

    while (
        rmw_uros_ping_agent_options(
            1000,
            1,
            rmw_options
        ) != RCL_RET_OK
    ) {
        printf("Agent no disponible. Reintentando...\n");
    }

    printf("Agent encontrado.\n");


    // Inicializar soporte de ROS 2
    RCCHECK(
        rclc_support_init_with_options(
            &support,
            0,
            NULL,
            &init_options,
            &allocator
        )
    );


    // ===============================
    // Nodo
    // ===============================

    rcl_node_t node;

    RCCHECK(
        rclc_node_init_default(
            &node,
            NODE_NAME,
            NODE_NAMESPACE,
            &support
        )
    );


    // ===============================
    // Executor
    // ===============================

    rclc_executor_t executor;

    /*
     * El ultimo parametro indica la cantidad maxima
     * de handles:
     *
     * subscriptions
     * timers
     * services
     * clients
     *
     * Cambiar este numero segun sea necesario.
     */
    RCCHECK(
        rclc_executor_init(
            &executor,
            &support.context,
            1,
            &allocator
        )
    );


    printf("\nmicro-ROS iniciado correctamente.\n");
    printf("Nodo: %s\n", NODE_NAME);
    printf("ROS_DOMAIN_ID: %d\n\n", ROS_DOMAIN_ID);


    // ===============================
    // Loop principal
    // ===============================

    while (1) {
        RCSOFTCHECK(
            rclc_executor_spin_some(
                &executor,
                RCL_MS_TO_NS(10)
            )
        );

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


// ===============================
// Entrada de ESP-IDF
// ===============================

void app_main(void)
{
    printf("\n=== PRUEBA MPU6050 VERSION 1 ===\n");
    fflush(stdout);

    mpu6050_config_t imu_config = {
        .sda_gpio = SDA,
        .scl_gpio = SCL,
        .i2c_frequency_hz = I2C_FRECUENCY_HZ,
        .address = I2C_ADDRESS,
    };

    ESP_ERROR_CHECK(mpu6050_init(&imu_config));

    uint8_t sensor_id = 0;

    ESP_ERROR_CHECK(mpu6050_read_id(&sensor_id));

    if (sensor_id != 0x68) {
        printf("Identificacion inesperada. Revisar el sensor.\n");
        return;
    }

    printf("Calibrando girómetro: no mover el sensor.\n");

    // Tiempo para soltar el módulo y dejarlo quieto.
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_ERROR_CHECK(
        mpu6050_calibrate_gyro(500, gyro_bias)
    );


    printf(
        "Offset crudo del girómetro: [%.2f, %.2f, %.2f]\n",
        gyro_bias[0],
        gyro_bias[1],
        gyro_bias[2]
    );

    BaseType_t imu_result = xTaskCreate(
        imu_task,
        "imu_task",
        4096,
        NULL,
        TASK_PRIORITY + 1,
        NULL
    );

    if (imu_result != pdPASS) {
        printf("No se pudo crear imu_task.\n");
        return;
    }


    quaternion_t q_test = {
        .w = 2.0f,
        .x = 0.0f,
        .y = 0.0f,
        .z = 2.0f
    };

    if (quaternion_normalize(&q_test) != 0) {
        printf("Error normalizando el cuaternion.\n");
        return;
    }

    quaternion_t identity = quaternion_multiply(
        q_test,
        quaternion_conjugate(q_test)
    );

    printf(
        "q normalizado: [%.4f, %.4f, %.4f, %.4f]\n",
        q_test.w, q_test.x, q_test.y, q_test.z
    );

    printf(
        "q * conjugado(q): [%.4f, %.4f, %.4f, %.4f]\n",
        identity.w, identity.x, identity.y, identity.z
    );


    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || \
        defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)

        uros_network_interface_initialize();

    #endif


    xTaskCreate(
        micro_ros_task,
        "micro_ros_task",
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY,
        NULL
    );
}