idf_component_register(
    SRCS
        "main.c"
        "mpu6050.c"
        "quaternion.c"
        "imu_filter.c"
        "imu_motion.c"

    INCLUDE_DIRS
        "."
        "include"

    REQUIRES
        driver
        esp_adc
        esp_driver_i2c
        micro_ros_espidf_component

    PRIV_REQUIRES
        esp_timer
        esp_driver_i2c
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)