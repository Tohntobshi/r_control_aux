#include <stdio.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "communication.h"
#include "control_loop.h"

void app_main(void)
{
    control_loop_setup();
    communication_setup();
}
