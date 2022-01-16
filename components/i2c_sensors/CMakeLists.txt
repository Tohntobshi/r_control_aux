idf_component_register(
    SRCS "i2c_sensors.c"
    INCLUDE_DIRS "include" "../"
    REQUIRES bmp280 VL53L1X utils
)
