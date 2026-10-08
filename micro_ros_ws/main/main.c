#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "esp_err.h"
#include "mpu6050.h"

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
                RCL_MS_TO_NS(100)
            )
        );

        usleep(10000);
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

    printf("MPU6050 WHO_AM_I: 0x%02X\n", (unsigned int)sensor_id);

    if (sensor_id != 0x68) {
        printf("Identificacion inesperada. Revisar el sensor.\n");
        return;
    }

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