#include <stdio.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "communication.h"
#include "motor_control/include/motor_control.h"
#include "us_sensor/include/us_sensor.h"

#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15
#define SPI_READY_PIN 27

static TaskHandle_t communicationTaskHandle;

static void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(SPI_READY_PIN, 1);
}

static void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(SPI_READY_PIN, 0);
}

static void communication_task(void * params)
{
    gpio_set_direction(SPI_READY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SPI_READY_PIN, 0);

    spi_bus_config_t buscfg = {
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode=0,
        .spics_io_num=GPIO_CS,
        .queue_size=1,
        .flags=0,
        .post_setup_cb=my_post_setup_cb,
        .post_trans_cb=my_post_trans_cb
    };

    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_DISABLED);

    spi_slave_transaction_t t;
    uint8_t sendbuf[9];
    uint8_t recvbuf[9];

    uint8_t current_command = 0; // 0 - wait for command, 1 - wait for motor vals, 2 - send current us duration
    
    while(1)
    {
        t.length = 9 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        if (current_command == 2)
        {
            uint16_t result = get_current_us_duration();
            sendbuf[0] = (uint8_t)(result >> 8); // msb
            sendbuf[1] = (uint8_t)(result); // lsb
            sendbuf[2] = (sendbuf[0] >> 4) + (sendbuf[0] & 0b00001111) + (sendbuf[1] >> 4) + (sendbuf[1] & 0b00001111); // checksum1
            sendbuf[3] = (~(sendbuf[0])) + (~(sendbuf[1])); // checksum2
        }

        ret = spi_slave_transmit(SPI2_HOST, &t, 5000 / portTICK_PERIOD_MS);

        if (t.trans_len == 8 && current_command == 0)
        {
            if (recvbuf[0] == 1) current_command = 1;
            if (recvbuf[0] == 2) current_command = 2;
        }
        else if (t.trans_len == 9 * 8 && current_command == 1) 
        {
            uint16_t fl = (uint16_t)(recvbuf[0]) << 8 | (uint16_t)(recvbuf[1]);
            uint16_t fr = (uint16_t)(recvbuf[2]) << 8 | (uint16_t)(recvbuf[3]);
            uint16_t bl = (uint16_t)(recvbuf[4]) << 8 | (uint16_t)(recvbuf[5]);
            uint16_t br = (uint16_t)(recvbuf[6]) << 8 | (uint16_t)(recvbuf[7]);
            uint8_t ch = (uint8_t)((fl >> 8) + fl + (fr >> 8) + fr + (bl >> 8) + bl + (br >> 8) + br);
            if (ch != recvbuf[8])
            {
                printf("motor checksum fail\n");
            }
            else
            {
                set_motor_vals(fl, fr, bl, br);
            }
            current_command = 0;
        }
        else
        {
            current_command = 0;
        }
    }
}

void communication_setup()
{
    xTaskCreate(communication_task, "communication_task", 5000, NULL, 1, &communicationTaskHandle);
}