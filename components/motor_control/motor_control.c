#include <stdio.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_control.h"

#define GPIO_FL_PIN 2
#define GPIO_FR_PIN 4
#define GPIO_BL_PIN 16
#define GPIO_BR_PIN 17

void motor_control_setup() {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_11_BIT, // resolution of PWM duty
        .freq_hz = 490,        // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel0 = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = GPIO_FL_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel0);
    ledc_channel_config_t ledc_channel1 = {
        .channel    = LEDC_CHANNEL_1,
        .duty       = 0,
        .gpio_num   = GPIO_FR_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel1);
    ledc_channel_config_t ledc_channel2 = {
        .channel    = LEDC_CHANNEL_2,
        .duty       = 0,
        .gpio_num   = GPIO_BL_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel2);
    ledc_channel_config_t ledc_channel3 = {
        .channel    = LEDC_CHANNEL_3,
        .duty       = 0,
        .gpio_num   = GPIO_BR_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel3);
}

void set_motor_vals(uint16_t fl_val, uint16_t fr_val, uint16_t bl_val, uint16_t br_val)
{
    uint32_t flduty = (fl_val / 2040.f) * 0b11111111111;
    uint32_t frduty = (fr_val / 2040.f) * 0b11111111111;
    uint32_t blduty = (bl_val / 2040.f) * 0b11111111111;
    uint32_t brduty = (br_val / 2040.f) * 0b11111111111;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, flduty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, frduty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, blduty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, brduty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);
}

void calibrate_esc()
{
    set_motor_vals(0, 0, 0, 0);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    set_motor_vals(2000, 2000, 2000, 2000);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    set_motor_vals(1000, 1000, 1000, 1000);
}