#include <stdio.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "communication.h"
#include "control_loop/include/control_loop.h"
#include "lwip/def.h"

#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15
#define SPI_READY_PIN 27

static TaskHandle_t communicationTaskHandle;

float get_float_from_net(uint8_t * data) {
    uint32_t tmp = ntohl(*(uint32_t *)(data));
    return *(float *)(&tmp);
}

int get_int_from_net(uint8_t * data) {
    return ntohl(*(uint32_t*)(data));
}

void set_float_to_net(float val, uint8_t * dest) {
    *(uint32_t *)dest = htonl(*(uint32_t *)(&val));
}

void set_short_to_net(uint16_t val, uint8_t * dest) {
    *(uint16_t *)dest = htons(val);
}

static uint8_t execute_read_command(uint8_t reg, uint8_t * buf_to_write_val) // 31 bytes max
{
    if (reg == GET_PITCH_AND_ROLL_INFO)
    {
        float err1 = get_current_pitch_err();
        float err_der1 = get_current_pitch_err_der();
        float err_der_int1 = get_pitch_err_int();
        float err2 = get_current_roll_err();
        float err_der2 = get_current_roll_err_der();
        float err_der_int2 = get_roll_err_int();
        set_float_to_net(err1, buf_to_write_val);
        set_float_to_net(err_der1, buf_to_write_val + 4);
        set_float_to_net(err_der_int1, buf_to_write_val + 8);
        set_float_to_net(err2, buf_to_write_val + 12);
        set_float_to_net(err_der2, buf_to_write_val + 16);
        set_float_to_net(err_der_int2, buf_to_write_val + 20);
        return 24;
    }
    if (reg == GET_YAW_AND_HEIGHT_INFO)
    {
        
        float err1 = get_current_yaw_err();
        float err_der1 = get_current_yaw_err_der();
        float err_der_int1 = get_yaw_err_int();
        float err2 = get_current_height_err();
        float err_der2 = get_current_height_err_der();
        float err_der_int2 = get_height_err_int();
        set_float_to_net(err1, buf_to_write_val);
        set_float_to_net(err_der1, buf_to_write_val + 4);
        set_float_to_net(err_der_int1, buf_to_write_val + 8);
        set_float_to_net(err2, buf_to_write_val + 12);
        set_float_to_net(err_der2, buf_to_write_val + 16);
        set_float_to_net(err_der_int2, buf_to_write_val + 20);
        return 24;
    }
    if (reg == GET_MOTOR_VALS_AND_FREQ)
    {
        uint16_t fl = get_fl_motor_val();
        uint16_t fr = get_fr_motor_val();
        uint16_t bl = get_bl_motor_val();
        uint16_t br = get_br_motor_val();
        float freq = get_loop_freq();
        set_short_to_net(fl, buf_to_write_val);
        set_short_to_net(fr, buf_to_write_val + 2);
        set_short_to_net(bl, buf_to_write_val + 4);
        set_short_to_net(br, buf_to_write_val + 6);
        set_float_to_net(freq, buf_to_write_val + 8);
        return 12;
    }
    if (reg == GET_GYRO_CALIBRATION)
    {
        vec3 vals;
        get_gyro_calibration(vals);
        set_float_to_net(vals[0], buf_to_write_val);
        set_float_to_net(vals[1], buf_to_write_val + 4);
        set_float_to_net(vals[2], buf_to_write_val + 8);
        return 12;
    }
    if (reg == GET_MAG_CALIBRATION)
    {
        
        vec3 mid, range;
        get_mag_calibration(mid, range);
        set_float_to_net(mid[0], buf_to_write_val);
        set_float_to_net(mid[1], buf_to_write_val + 4);
        set_float_to_net(mid[2], buf_to_write_val + 8);
        set_float_to_net(range[0], buf_to_write_val + 12);
        set_float_to_net(range[1], buf_to_write_val + 16);
        set_float_to_net(range[2], buf_to_write_val + 20);
        return 24;
    }
    return 0;
}

static void execute_write_command(uint8_t reg, uint8_t * buf_with_val, uint8_t size) // 31 bytes max
{
    if (reg == MOVE && size == 8)
    {
        float x = get_float_from_net(buf_with_val);
        float y = get_float_from_net(buf_with_val + 4);
        // printf("set move x %f y %f\n", x, y);
        set_move_vector(x, y);
        return;
    }
    if (reg == SET_DIRECTION && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set direction %f\n", res);
        set_desired_direction(res);
        return;
    }
    if (reg == SET_HEIGHT && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set height %f\n", res);
        set_desired_height(res);
        return;
    }
    if (reg == SET_BASE_ACCELERATION && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set base accel %f\n", res);
        set_base_acceleration(res);
        return;
    }
    if (reg == SET_PITCH_PROP_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set pitch prop coef %f\n", res);
        set_pitch_prop_coef(res);
        return;
    }
    if (reg == SET_PITCH_DER_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set pitch der coef %f\n", res);
        set_pitch_der_coef(res);
        return;
    }
    if (reg == SET_PITCH_INT_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set pitch int coef %f\n", res);
        set_pitch_int_coef(res);
        return;
    }
    if (reg == SET_ROLL_PROP_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set roll prop coef %f\n", res);
        set_roll_prop_coef(res);
        return;
    }
    if (reg == SET_ROLL_DER_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set roll der coef %f\n", res);
        set_roll_der_coef(res);
        return;
    }
    if (reg == SET_ROLL_INT_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set roll int coef %f\n", res);
        set_roll_int_coef(res);
        return;
    }
    if (reg == SET_YAW_PROP_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set yaw prop coef %f\n", res);
        set_yaw_prop_coef(res);
        return;
    }
    if (reg == SET_YAW_DER_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set yaw der coef %f\n", res);
        set_yaw_der_coef(res);
        return;
    }
    if (reg == SET_YAW_INT_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set yaw int coef %f\n", res);
        set_yaw_int_coef(res);
        return;
    }
    if (reg == SET_HEIGHT_PROP_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set height prop coef %f\n", res);
        set_height_prop_coef(res);
        return;
    }
    if (reg == SET_HEIGHT_DER_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set height der coef %f\n", res);
        set_height_der_coef(res);
        return;
    }
    if (reg == SET_HEIGHT_INT_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set height int coef %f\n", res);
        set_height_int_coef(res);
        return;
    }
    if (reg == SET_ACC_TRUST && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set acc trust %f\n", res);
        set_acc_trust(res);
        return;
    }
    if (reg == SET_MAG_TRUST && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set mag trust %f\n", res);
        set_mag_trust(res);
        return;
    }
    if (reg == SET_IMU_LPF_MODE && size == 1)
    {
        // printf("set lpf mode %d\n", buf_with_val[0]);
        schedule_set_acc_gyro_filtering_mode(buf_with_val[0]);
        return;
    }
    if (reg == RESET_TURN_OFF_TRIGGER && size == 1)
    {
        // printf("reset turn off\n");
        reset_turn_off_trigger();
        return;
    }
    if (reg == SET_TURN_OFF_INCLINE_ANGLE && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set turn off angle%f\n", res);
        set_turn_off_incline_angle(res);
        return;
    }
    if (reg == SET_PITCH_ADJUST && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set pitch adj %f\n", res);
        set_pitch_adjust(res);
        return;
    }
    if (reg == SET_ROLL_ADJUST && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        // printf("set roll adj %f\n", res);
        set_roll_adjust(res);
        return;
    }
    if (reg == CALIBRATE_GYRO && size == 1)
    {
        // printf("calibrate gyro\n");
        schedule_gyro_calibration();
        return;
    }
    if (reg == CALIBRATE_MAG && size == 1)
    {
        // printf("calibrate mag\n");
        schedule_mag_calibration();
        return;
    }
    if (reg == CALIBRATE_ESC && size == 1)
    {
        // printf("calibrate esc\n");
        schedule_esc_calibration();
        return;
    }
    if (reg == SET_GYRO_CALIBRATION && size == 12)
    {
        vec3 result;
        result[0] = get_float_from_net(buf_with_val);
        result[1] = get_float_from_net(buf_with_val + 4);
        result[2] = get_float_from_net(buf_with_val + 8);
        // printf("set gyro calib %f %f %f\n", result[0], result[1], result[2]);
        set_gyro_calibration(result);
        return;
    }
    if (reg == SET_MAG_CALIBRATION && size == 24)
    {
        vec3 mid;
        vec3 range;
        mid[0] = get_float_from_net(buf_with_val);
        mid[1] = get_float_from_net(buf_with_val + 4);
        mid[2] = get_float_from_net(buf_with_val + 8);
        range[0] = get_float_from_net(buf_with_val + 12);
        range[1] = get_float_from_net(buf_with_val + 16);
        range[2] = get_float_from_net(buf_with_val + 20);
        // printf("set mag calib mid %f %f %f range %f %f %f\n", mid[0], mid[1], mid[2], range[0], range[1], range[2]);
        set_mag_calibration(mid, range);
        return;
    }
}

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
    uint8_t sendbuf[32];
    uint8_t recvbuf[32];

    uint8_t command_type = 0; // 0 - wait for command, 1 - write, 2 - read
    uint8_t reg = 0;

    while(1)
    {
        t.length = 32 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        if (command_type == 0)
        {
            t.length = 8;
        }
        if (command_type == 2)
        {
            uint8_t length = execute_read_command(reg, sendbuf);
            sendbuf[length] = reg; // crc
            for(int i = 0; i < length; i++)
            {
                sendbuf[length] = sendbuf[length] ^ sendbuf[i];
            }
            t.length = 8 * (length + 1);
        }

        ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);

        if (t.trans_len == 8 && command_type == 0)
        {
            if (recvbuf[0] == 0)
            {
                command_type = 0;
                continue;
            }
            if (recvbuf[0] & 0b10000000) {
                command_type = 2;
                reg = recvbuf[0] & 0b01111111;
                continue;
            }
            command_type = 1;
            reg = recvbuf[0];
            continue;
        }
        else if (t.trans_len >= 16 && command_type == 1) 
        {
            uint8_t length = t.trans_len / 8 - 1;
            uint8_t crc = reg;
            for(int j = 0; j < length; j++)
            {
                crc = crc ^ recvbuf[j];
            }
            if (crc == recvbuf[length])
            {
                execute_write_command(reg, recvbuf, length);
            }
            command_type = 0;
        }
        else
        {
            command_type = 0;
        }
    }
}

void communication_setup()
{
    xTaskCreate(communication_task, "communication_task", 5000, NULL, 1, &communicationTaskHandle);
}