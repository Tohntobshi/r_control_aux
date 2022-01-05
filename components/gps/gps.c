#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "gps.h"


#define TX_PIN 22
#define RX_PIN 23

static TaskHandle_t gpsTaskHandle;

static uint8_t actual_data = 0;
static float latitude = 0.f;
static float longitude = 0.f;
static int satelites = 0.f;

float get_latitude()
{
    return latitude;
}

float get_longitude()
{
    return longitude;
}

int get_num_satelites()
{
    return satelites;
}

uint8_t get_lat_lon_actuality()
{
    return actual_data;
}

static int extract_portion(uint8_t * src, uint8_t * dst, int max)
{
    int count = 0;
    while (1)
    {
        if (src[count] == ',' || src[count] == '\0' || src[count] == '*' || count == max) return count;
        dst[count] = src[count];
        count++;
    }
}

static float parse_to_degrees(uint8_t* data)
{
    float degrees;
    float minutes;
    int length = strlen((char *)data);
    if (!length) return 0.f;
    uint8_t * minutes_ptr = (uint8_t *)strchr((char *)data, '.');
    if (minutes_ptr == NULL) return 0.f;
    minutes_ptr -= 2;
    if (minutes_ptr < data) return 0.f;
    sscanf((char *)minutes_ptr, "%f", &minutes);
    int degree_str_length = (int)(minutes_ptr - data) + 1;
    uint8_t degrees_str[degree_str_length];
    for (int i = 0; i < degree_str_length - 1; i++)
    {
        degrees_str[i] = data[i];
    }
    degrees_str[degree_str_length] = '\0';
    sscanf((char *)degrees_str, "%f", &degrees);
    return degrees + minutes / 60.f;
}

static void gps_task(void * params)
{
    uart_driver_install(UART_NUM_2, 256, 0, 10, NULL, 0);
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    while (1)
    {
        uint8_t data[257] = { 0 };
        int length = 256;
        length = uart_read_bytes(UART_NUM_2, data, length, 50 / portTICK_RATE_MS);
        if (length < 10) {
            continue;
        }
        // printf("raw %dbytes %s\n", length, data);
        if (strncmp("$GNGGA,", (char *)data, 7) != 0)
        {
            printf("gps some garbage \n");
            continue;
        }
        printf((char *)data);
        int parsed_count = 7;
        uint8_t time_str[16] = { 0 };
        uint8_t lat_str[16] = { 0 };
        uint8_t ns_str[2] = { 0 };
        uint8_t lon_str[16] = { 0 };
        uint8_t ew_str[2] = { 0 };
        uint8_t qual_str[2] = { 0 };
        uint8_t num_sat_str[4] = { 0 };
        uint8_t HDOP_str[10] = { 0 };
        uint8_t alt_str[16] = { 0 };
        uint8_t m_str[2] = { 0 };
        uint8_t geo_sep_str[16] = { 0 };
        uint8_t m2_str[2] = { 0 };
        uint8_t unknown_str[16] = { 0 };
        uint8_t DGPS_station_ID_str[16] = { 0 };
        uint8_t check_str[4] = { 0 };
        parsed_count += 1 + extract_portion(data + parsed_count, time_str, 15);
        parsed_count += 1 + extract_portion(data + parsed_count, lat_str, 15);
        parsed_count += 1 + extract_portion(data + parsed_count, ns_str, 1);
        parsed_count += 1 + extract_portion(data + parsed_count, lon_str, 15);
        parsed_count += 1 + extract_portion(data + parsed_count, ew_str, 1);
        parsed_count += 1 + extract_portion(data + parsed_count, qual_str, 1);
        parsed_count += 1 + extract_portion(data + parsed_count, num_sat_str, 4);
        parsed_count += 1 + extract_portion(data + parsed_count, HDOP_str, 9);
        parsed_count += 1 + extract_portion(data + parsed_count, alt_str, 15);
        parsed_count += 1 + extract_portion(data + parsed_count, m_str, 1);
        parsed_count += 1 + extract_portion(data + parsed_count, geo_sep_str, 15);
        parsed_count += 1 + extract_portion(data + parsed_count, m2_str, 1);
        parsed_count += 1 + extract_portion(data + parsed_count, unknown_str, 15);
        parsed_count += 1 + extract_portion(data + parsed_count, DGPS_station_ID_str, 15);
        uint8_t estimated_checksum = 0;
        for (int i = 1; i < parsed_count - 1; i++)
        {
            estimated_checksum ^= data[i];
        }
        extract_portion(data + parsed_count, check_str, 3);
        uint32_t received_checksum;
        sscanf((char *)check_str, "%X", &received_checksum);
        if (estimated_checksum != received_checksum)
        {
            printf("gps some garbage checksum \n");
            continue;
        }
        uint8_t actual_data_flag = 1;
        if (strlen((char *)lat_str))
        {
            latitude = parse_to_degrees(lat_str) * (ns_str[0] == 'S' ? -1.f : 1.f);
        }
        else
        {
            actual_data_flag = 0;
        }
        if (strlen((char *)lon_str))
        {
            longitude = parse_to_degrees(lon_str) * (ew_str[0] == 'W' ? -1.f : 1.f);
        }
        else
        {
            actual_data_flag = 0;
        }
        if (strlen((char *)num_sat_str)) {
            sscanf((char *)num_sat_str, "%d", &satelites);
        }
        else
        {
            actual_data_flag = 0;
        }
        actual_data = actual_data_flag;
    }
}


void gps_setup()
{
    xTaskCreate(gps_task, "gps_task", 50000, NULL, 1, &gpsTaskHandle);
}
