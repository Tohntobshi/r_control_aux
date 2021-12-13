#include <stdio.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "motor_control.h"
#include "us_sensor.h"
#include "communication.h"


void app_main(void)
{
    motor_control_setup();
    // calibrate_esc();
    ultrasonic_setup();
    communication_setup();
}
