idf_component_register(
    SRCS "control_loop.c"
    INCLUDE_DIRS "include" "../"
    REQUIRES i2c_sensors us_sensor motor_control
)
