#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include "freertos/queue.h"
#include "geometry_msgs/msg/pose_stamped.h"
#include "rosidl_runtime_c/string_functions.h"

#include "esp_err.h"
#include "mpu6050.h"
#include "esp_timer.h"
#include "quaternion.h"
#include "imu_filter.h"


#include <rmw_microros/rmw_microros.h>

// ===============================
// Estructuras
// ===============================
typedef struct {
    int64_t time_us;
    quaternion_t orientation;
} attitude_sample_t;

static QueueHandle_t attitude_queue;


#define ROS_DOMAIN_ID     10

#define SDA               21
#define SCL               22
#define I2C_FRECUENCY_HZ  100000
#define I2C_ADDRESS       0x68
#define ALPHA             0.98f 


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

    imu_filter_t filter;

    if (imu_filter_init(&filter, ALPHA) != 0) {
        printf("Error inicializando el filtro.\n");
        vTaskDelete(NULL);
        return;
    }

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
        int attitude_valid = 0;

        if (err == ESP_OK) {
            for (int i = 0; i < 3; i++) {
                accel[i] = (raw.accel[i] - accel_bias[i])
                        * (9.80665f / accel_sensitivity[i]);

                gyro[i] = (raw.gyro[i] - gyro_bias[i])
                        * (0.01745329252f / 131.0f);
            }

            float dt = (float)dt_us * 1e-6f;

            if (imu_filter_update(&filter, accel, gyro, dt) != 0) {
                printf("Error actualizando el filtro.\n");
            } else {
                attitude_valid = 1;

                attitude_sample_t sample = {
                    .time_us = now,
                    .orientation = filter.orientation
                };

                xQueueOverwrite(attitude_queue, &sample);
            }

        }

        intervals++;

        if (intervals >= 200) {

            if (err == ESP_OK) {


                if (attitude_valid) {
                    printf(
                        "q filtrado [w,x,y,z]: [%.4f, %.4f, %.4f, %.4f]\n"
                        "errores I2C: %lu\n",
                        filter.orientation.w,
                        filter.orientation.x,
                        filter.orientation.y,
                        filter.orientation.z,
                        (unsigned long)errors
                    );
                }

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


    while (
        rmw_uros_ping_agent_options(
            1000,
            1,
            rmw_options
        ) != RCL_RET_OK
    ) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

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


    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();

    rcl_ret_t pub_rc = rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            geometry_msgs, msg, PoseStamped),
        "/imu_pose"
    );

    if (pub_rc != RCL_RET_OK) {
        printf("Error creando publisher: rc=%d\n", (int)pub_rc);

        if (rcl_error_is_set()) {
            printf("Detalle: %s\n", rcl_get_error_string().str);
        }

        fflush(stdout);
        vTaskDelete(NULL);
        return;
    }

    printf("Publisher creado.\n");

    geometry_msgs__msg__PoseStamped msg = {0};

    if (!geometry_msgs__msg__PoseStamped__init(&msg) ||
        !rosidl_runtime_c__String__assign(
            &msg.header.frame_id, "world")) {
        printf("Error inicializando mensaje de actitud.\n");
        vTaskDelete(NULL);
        return;
    }

    // Posición fija: visualizamos solamente orientación.
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;

    if (rmw_uros_sync_session(1000) != RCL_RET_OK) {
        printf("Error sincronizando reloj.\n");
        vTaskDelete(NULL);
        return;
    }

    int64_t epoch_offset_ns =
        rmw_uros_epoch_nanos() -
        esp_timer_get_time() * 1000LL;

    printf("Publicando actitud en /imu_pose.\n");

    TickType_t publish_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(
            &publish_wake,
            pdMS_TO_TICKS(20)
        );

        attitude_sample_t sample;

        if (xQueueReceive(attitude_queue, &sample, 0) != pdTRUE) {
            continue;
        }

        int64_t stamp_ns =
            epoch_offset_ns + sample.time_us * 1000LL;

        msg.header.stamp.sec =
            (int32_t)(stamp_ns / 1000000000LL);

        msg.header.stamp.nanosec =
            (uint32_t)(stamp_ns % 1000000000LL);

        msg.pose.orientation.w = sample.orientation.w;
        msg.pose.orientation.x = sample.orientation.x;
        msg.pose.orientation.y = sample.orientation.y;
        msg.pose.orientation.z = sample.orientation.z;

        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }
}


// ===============================
// Entrada de ESP-IDF
// ===============================

void app_main(void)
{
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


    
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || \
    defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    
    uros_network_interface_initialize();
    
    #endif
    

    attitude_queue = xQueueCreate(1, sizeof(attitude_sample_t));

    if (attitude_queue == NULL) {
        printf("Error creando cola de actitud.\n");
        return;
    }

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

    xTaskCreate(
        micro_ros_task,
        "micro_ros_task",
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY,
        NULL
    );
}