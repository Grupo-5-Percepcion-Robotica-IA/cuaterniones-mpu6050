#include <stdio.h>
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
#include "imu_filter.h"
#include "imu_motion.h"

#include <rmw_microros/rmw_microros.h>

#include "freertos/queue.h"
#include "sensor_msgs/msg/imu.h"
#include "geometry_msgs/msg/pose_stamped.h"
#include "rosidl_runtime_c/string_functions.h"

// ===============================
// Estructuras para publicar
// ===============================

typedef struct {
    int64_t time_us;
    float accel[3];
    float gyro[3];
    quaternion_t orientation;
    float position[3];
} imu_snapshot_t;

static QueueHandle_t imu_queue;

// ===============================
// Configuracion general
// ===============================

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

    //Arrancamos con posición y velocidad -> 0
    imu_motion_t motion;
    imu_motion_reset(&motion);

    float linear_bias[3] = {0.0f, 0.0f, 0.0f};
    double linear_sum[3] = {0.0, 0.0, 0.0};

    float accel_corrected[3] = {0.0f, 0.0f, 0.0f};

    uint32_t linear_samples = 0;
    int linear_calibrated = 0;
    int64_t linear_start_us = 0;

    printf("Calibrando residuo lineal: mantener quieto 5 segundos.\n");

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
        float accel_world[3];
        float accel_linear[3];
        
        int world_valid = 0;

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
            } else if (quaternion_rotate_vector(
                        filter.orientation,
                        accel,
                        accel_world) != 0) {
                printf("Error rotando la aceleracion.\n");
            } else {
                // Mundo con Z hacia arriba:
                // en reposo, la lectura rotada es aproximadamente [0, 0, +g].
                accel_linear[0] = accel_world[0];
                accel_linear[1] = accel_world[1];
                accel_linear[2] = accel_world[2] - 9.80665f;

                world_valid = 1;


                if (!linear_calibrated) {
                    if (linear_start_us == 0) {
                        linear_start_us = now;
                    }

                    int64_t elapsed_us = now - linear_start_us;

                    // Primeros 2 segundos: dejar estabilizar el filtro.
                    if (elapsed_us >= 2000000LL) {
                        for (int i = 0; i < 3; i++) {
                            linear_sum[i] += accel_linear[i];
                        }

                        linear_samples++;

                        // Siguientes 3 segundos: promediar muestras válidas.
                        if (elapsed_us >= 5000000LL &&
                            linear_samples >= 500) {

                            for (int i = 0; i < 3; i++) {
                                linear_bias[i] =
                                    (float)(linear_sum[i] / linear_samples);
                            }

                            imu_motion_reset(&motion);
                            linear_calibrated = 1;

                            printf(
                                "Residuo lineal [m/s²]: [%.4f, %.4f, %.4f]\n",
                                linear_bias[0],
                                linear_bias[1],
                                linear_bias[2]
                            );

                            printf("Calibracion lista. Comienza la integracion.\n");
                        }
                    }
                } else {
                    for (int i = 0; i < 3; i++) {
                        accel_corrected[i] = accel_linear[i] - linear_bias[i];
                    }

                    if (imu_motion_update(
                            &motion,
                            accel_corrected,
                            dt
                        ) != 0) {
                        printf("Error integrando aceleracion.\n");
                    }

                    imu_snapshot_t snapshot = {
                        .time_us = now,
                        .orientation = filter.orientation
                    };

                    for (int i = 0; i < 3; i++) {
                        snapshot.accel[i] = accel[i];
                        snapshot.gyro[i] = gyro[i];
                        snapshot.position[i] = motion.position[i];
                    }

                    xQueueOverwrite(imu_queue, &snapshot);
                }
            }

        }

        intervals++;

        if (intervals >= 200) {
            // printf(
            //     "IMU: dt promedio=%.1f us | "
            //     "min=%lld | max=%lld | errores=%lu\n",
            //     (double)total_dt / intervals,
            //     (long long)min_dt,
            //     (long long)max_dt,
            //     (unsigned long)errors
            // );

            if (err == ESP_OK) {

                // printf(
                //     "ACC [m/s²]: [%.2f, %.2f, %.2f] | "
                //     "GYRO [rad/s]: [%.4f, %.4f, %.4f]\n",
                //     accel[0], accel[1], accel[2],
                //     gyro[0], gyro[1], gyro[2]
                // );

                if (world_valid && linear_calibrated) {
                    printf(
                        "ACC corregida [m/s²]: [%.3f, %.3f, %.3f]\n"
                        "VEL [m/s]: [%.3f, %.3f, %.3f]\n"
                        "POS [m]: [%.3f, %.3f, %.3f]\n",
                        accel_corrected[0],
                        accel_corrected[1],
                        accel_corrected[2],
                        motion.velocity[0],
                        motion.velocity[1],
                        motion.velocity[2],
                        motion.position[0],
                        motion.position[1],
                        motion.position[2]
                    );
                }

                // printf(
                //     "q filtrado: [%.4f, %.4f, %.4f, %.4f]\n",
                //     filter.orientation.w,
                //     filter.orientation.x,
                //     filter.orientation.y,
                //     filter.orientation.z
                // );
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

    // ===============================
    // Publicadores
    // ===============================
    rcl_publisher_t imu_publisher = rcl_get_zero_initialized_publisher();

    rcl_publisher_t pose_publisher = rcl_get_zero_initialized_publisher();

    RCCHECK(rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu_data_rcv"
    ));

    RCCHECK(rclc_publisher_init_best_effort(
        &pose_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
        "/imu_pose"
    ));

    sensor_msgs__msg__Imu imu_msg = {0};
    geometry_msgs__msg__PoseStamped pose_msg = {0};

    if (!sensor_msgs__msg__Imu__init(&imu_msg) ||
        !geometry_msgs__msg__PoseStamped__init(&pose_msg)) {
        printf("Error inicializando mensajes.\n");
        vTaskDelete(NULL);
        return;
    }

    if (!rosidl_runtime_c__String__assign(
            &imu_msg.header.frame_id, "imu_link") ||
        !rosidl_runtime_c__String__assign(
            &pose_msg.header.frame_id, "world")) {
        printf("Error asignando frames.\n");
        vTaskDelete(NULL);
        return;
    }

    // Este tópico no contiene una estimación de orientación.
    imu_msg.orientation.w = 1.0;
    imu_msg.orientation_covariance[0] = -1.0;

    // Sincronizar reloj con el Agent.
    if (rmw_uros_sync_session(1000) != RCL_RET_OK) {
        printf("Error sincronizando reloj con el Agent.\n");
        vTaskDelete(NULL);
        return;
    }

    // Convertir tiempos monotónicos de adquisición a tiempo ROS.
    int64_t epoch_offset_ns =
        rmw_uros_epoch_nanos() - esp_timer_get_time() * 1000LL;


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

    TickType_t publish_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&publish_wake, pdMS_TO_TICKS(20));

        imu_snapshot_t snapshot;

        if (xQueueReceive(imu_queue, &snapshot, 0) != pdTRUE) {
            continue;
        }

        int64_t stamp_ns =
            epoch_offset_ns + snapshot.time_us * 1000LL;

        imu_msg.header.stamp.sec =
            (int32_t)(stamp_ns / 1000000000LL);

        imu_msg.header.stamp.nanosec =
            (uint32_t)(stamp_ns % 1000000000LL);

        pose_msg.header.stamp = imu_msg.header.stamp;

        // Datos expresados en los ejes del sensor.
        imu_msg.linear_acceleration.x = snapshot.accel[0];
        imu_msg.linear_acceleration.y = snapshot.accel[1];
        imu_msg.linear_acceleration.z = snapshot.accel[2];

        imu_msg.angular_velocity.x = snapshot.gyro[0];
        imu_msg.angular_velocity.y = snapshot.gyro[1];
        imu_msg.angular_velocity.z = snapshot.gyro[2];

        // Pose expresada en los ejes del mundo.
        pose_msg.pose.position.x = 0.0;
        pose_msg.pose.position.y = 0.0;
        pose_msg.pose.position.z = 0.0;

        pose_msg.pose.orientation.w = snapshot.orientation.w;
        pose_msg.pose.orientation.x = snapshot.orientation.x;
        pose_msg.pose.orientation.y = snapshot.orientation.y;
        pose_msg.pose.orientation.z = snapshot.orientation.z;

        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
        RCSOFTCHECK(rcl_publish(&pose_publisher, &pose_msg, NULL));
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


    // printf(
    //     "Offset crudo del girómetro: [%.2f, %.2f, %.2f]\n",
    //     gyro_bias[0],
    //     gyro_bias[1],
    //     gyro_bias[2]
    // );

    
    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || \
    defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    
    uros_network_interface_initialize();
    
    #endif
    
    imu_queue = xQueueCreate(1, sizeof(imu_snapshot_t));

    if (imu_queue == NULL) {
        printf("Error creando la cola IMU.\n");
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