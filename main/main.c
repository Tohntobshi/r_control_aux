#include <stdio.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"
#include "us_sensor.h"
#include "communication.h"
#include "i2c_sensors.h"


void app_main(void)
{
    motor_control_setup();
    // calibrate_esc();
    ultrasonic_setup();
    communication_setup();
    i2c_sensors_setup();

    // while (1)
    // {
    //     float val = get_bar_data();
    //     vec3 gy;
    //     get_gyro_calibrated_data(gy);
    //     printf("bar %f gy %f %f %f\n", val, gy[0], gy[1], gy[2]);
    //     vTaskDelay(50 / portTICK_PERIOD_MS);
    // }
}
