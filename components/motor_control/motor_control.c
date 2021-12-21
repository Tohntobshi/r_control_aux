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
        .freq_hz = 3333,        // frequency of PWM signal
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

static void disable_motor_control()
{
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, 0);
}

void set_motor_vals(float fl_val, float fr_val, float bl_val, float br_val)
{
    float fl_duty_fraction = (fl_val * 125.f + 125.f) / 300.f; // from 125us to 250us out of 300us full cycle
    float fr_duty_fraction = (fr_val * 125.f + 125.f) / 300.f; // from 125us to 250us out of 300us full cycle
    float bl_duty_fraction = (bl_val * 125.f + 125.f) / 300.f; // from 125us to 250us out of 300us full cycle
    float br_duty_fraction = (br_val * 125.f + 125.f) / 300.f; // from 125us to 250us out of 300us full cycle
    uint32_t flduty = fl_duty_fraction * 0b11111111111;
    uint32_t frduty = fr_duty_fraction * 0b11111111111;
    uint32_t blduty = bl_duty_fraction * 0b11111111111;
    uint32_t brduty = br_duty_fraction * 0b11111111111;
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
    disable_motor_control();
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    motor_control_setup();
    set_motor_vals(1.0f, 1.0f, 1.0f, 1.0f);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    set_motor_vals(0.0f, 0.0f, 0.0f, 0.0f);
}