#include <stdio.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "communication.h"
#include "control_loop.h"
#include "battery_check.h"
#include "storage.h"
#include "gps.h"

void app_main(void)
{
    storage_init();
    gps_setup();
    battery_check_setup();
    control_loop_setup();
    communication_setup();
}
