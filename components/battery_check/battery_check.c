#include <stdio.h>
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "battery_check.h"
#include "esp_adc_cal.h"


static TaskHandle_t batteryCheckTaskHandle;

#ifndef VOLTAGE_SHIFT_FOR_THIS_PARTICULAR_DEVICE
#define VOLTAGE_SHIFT_FOR_THIS_PARTICULAR_DEVICE 0.4f
#endif

static float current_voltage = 0.f;

float get_current_voltage()
{
    return current_voltage + VOLTAGE_SHIFT_FOR_THIS_PARTICULAR_DEVICE;
}

static void battery_check_task(void * params)
{
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    esp_adc_cal_characteristics_t adc_chars; // = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 1100, &adc_chars);
    while (1) 
    {
        int raw = adc1_get_raw(ADC1_CHANNEL_7);
        current_voltage = (esp_adc_cal_raw_to_voltage(raw, &adc_chars) / 1000.f) * 5.579f * 0.2f + current_voltage * 0.8f ;
        // printf("adc %f V\n", voltage);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
}

void battery_check_setup()
{
    xTaskCreate(battery_check_task, "battery_check_task", 50000, NULL, 1, &batteryCheckTaskHandle);
}