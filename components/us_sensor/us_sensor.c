#include <stdio.h>
#include "driver/gpio.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "us_sensor.h"

#define US_SENSOR_TRIG_PIN 26
#define US_SENSOR_ECHO_PIN 25

volatile static uint16_t current_duration = 0;
static TaskHandle_t ultrasonicTaskHandle;
static QueueHandle_t xQueue;

static void IRAM_ATTR gpio_echo_pin_handler(void* arg)
{
    if (gpio_get_level(US_SENSOR_ECHO_PIN))
    {
        timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    }
    else
    {
        uint64_t end_timestamp = 0;
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &end_timestamp);
        uint16_t result = end_timestamp;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(xQueue, &result, &xHigherPriorityTaskWoken);
        if( xHigherPriorityTaskWoken )
        {
            portYIELD_FROM_ISR ();
        }
    }
}

static void ultrasonic_task(void * params)
{
    timer_config_t t_conf = {
        .divider = 80,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_START,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &t_conf);


    gpio_set_direction(US_SENSOR_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(US_SENSOR_TRIG_PIN, 0);

    gpio_set_direction(US_SENSOR_ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(US_SENSOR_ECHO_PIN, GPIO_INTR_ANYEDGE);
    gpio_intr_enable(US_SENSOR_ECHO_PIN);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(US_SENSOR_ECHO_PIN, gpio_echo_pin_handler, NULL);
    
    while(1)
    {
        uint16_t val;
        if (xQueueReceive(xQueue, &val, 1000 / portTICK_PERIOD_MS))
        {
            current_duration = val;
        }
        else
        {
            current_duration = 24000; // 4m
        }
        gpio_set_level(US_SENSOR_TRIG_PIN, 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        gpio_set_level(US_SENSOR_TRIG_PIN, 0);
    }
}

uint16_t get_current_us_duration()
{
    return current_duration;    // 16bit value on 32bit system doesn't require mutex (i hope)
}

void ultrasonic_setup()
{
    xQueue = xQueueCreate( 1, sizeof( uint16_t ) );
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 5000, NULL, 1, &ultrasonicTaskHandle);
}