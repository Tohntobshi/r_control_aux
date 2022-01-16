#include <stdio.h>
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "communication.h"
#include "control_loop/include/control_loop.h"
#include "gps/include/gps.h"
#include "utils.h"

#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15
#define SPI_READY_PIN 27

// #define PRINT_ALL_SET_COMMANDS 1

static TaskHandle_t communicationTaskHandle;

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
    if (reg == GET_MOTOR_VALS_FREQ_AND_VOLTAGE)
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
        set_float_to_net(get_base_voltage(), buf_to_write_val + 12);
        return 16;
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
    if (reg == GET_ACC_CALIBRATION)
    {
        vec3 vals;
        get_acc_calibration(vals);
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
    if (reg == GET_PRIMARY_INFO)
    {
        buf_to_write_val[0] = get_landing_flag();
        set_float_to_net(get_base_voltage(), buf_to_write_val + 1);
        buf_to_write_val[5] = get_lat_lon_actuality();
        set_short_to_net(get_num_satelites(), buf_to_write_val + 6);
        return 8;
    }
    if (reg == GET_POSITION_INFO)
    {
        set_float_to_net(get_x_position_err(), buf_to_write_val);
        set_float_to_net(get_y_position_err(), buf_to_write_val + 4);
        set_float_to_net(get_x_position_err_der(), buf_to_write_val + 8);
        set_float_to_net(get_y_position_err_der(), buf_to_write_val + 12);
        set_float_to_net(get_x_position_err_int(), buf_to_write_val + 16);
        set_float_to_net(get_y_position_err_int(), buf_to_write_val + 20);
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
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set move x %f y %f\n", x, y);
        #endif
        set_move_vector(x, y);
        return;
    }
    if (reg == SET_DIRECTION && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set direction %f\n", res);
        #endif
        set_desired_direction(res);
        return;
    }
    if (reg == SET_HEIGHT && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set height %f\n", res);
        #endif
        set_desired_height(res);
        return;
    }
    if (reg == SET_BASE_ACCELERATION && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set base accel %f\n", res);
        #endif
        set_base_acceleration(res);
        return;
    }
    if (reg == SET_PITCH_PROP_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set pitch prop coef %f\n", res);
        #endif
        set_pitch_prop_coef(res);
        return;
    }
    if (reg == SET_PITCH_DER_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set pitch der coef %f\n", res);
        #endif
        set_pitch_der_coef(res);
        return;
    }
    if (reg == SET_PITCH_INT_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set pitch int coef %f\n", res);
        #endif
        set_pitch_int_coef(res);
        return;
    }
    if (reg == SET_ROLL_PROP_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set roll prop coef %f\n", res);
        #endif
        set_roll_prop_coef(res);
        return;
    }
    if (reg == SET_ROLL_DER_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set roll der coef %f\n", res);
        #endif
        set_roll_der_coef(res);
        return;
    }
    if (reg == SET_ROLL_INT_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set roll int coef %f\n", res);
        #endif
        set_roll_int_coef(res);
        return;
    }
    if (reg == SET_YAW_PROP_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set yaw prop coef %f\n", res);
        #endif
        set_yaw_prop_coef(res);
        return;
    }
    if (reg == SET_YAW_DER_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set yaw der coef %f\n", res);
        #endif
        set_yaw_der_coef(res);
        return;
    }
    if (reg == SET_YAW_INT_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set yaw int coef %f\n", res);
        #endif
        set_yaw_int_coef(res);
        return;
    }
    if (reg == SET_HEIGHT_PROP_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set height prop coef %f\n", res);
        #endif
        set_height_prop_coef(res);
        return;
    }
    if (reg == SET_HEIGHT_DER_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set height der coef %f\n", res);
        #endif
        set_height_der_coef(res);
        return;
    }
    if (reg == SET_HEIGHT_INT_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set height int coef %f\n", res);
        #endif
        set_height_int_coef(res);
        return;
    }
    if (reg == SET_ACC_TRUST && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set acc trust %f\n", res);
        #endif
        set_acc_trust(res);
        return;
    }
    if (reg == SET_MAG_TRUST && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set mag trust %f\n", res);
        #endif
        set_mag_trust(res);
        return;
    }
    if (reg == SET_ACC_LPF_MODE && size == 1)
    {
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set acc lpf mode %d\n", buf_with_val[0]);
        #endif
        schedule_set_acc_filtering_mode(buf_with_val[0]);
        return;
    }
     if (reg == SET_GYRO_LPF_MODE && size == 1)
    {
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set gyro lpf mode %d\n", buf_with_val[0]);
        #endif
        schedule_set_gyro_filtering_mode(buf_with_val[0]);
        return;
    }
    if (reg == RESET_TURN_OFF_TRIGGER && size == 1)
    {
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("reset turn off\n");
        #endif
        reset_turn_off_trigger();
        return;
    }
    if (reg == SET_TURN_OFF_INCLINE_ANGLE && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set turn off angle %f\n", res);
        #endif
        set_turn_off_incline_angle(res);
        return;
    }
    if (reg == SET_PITCH_ADJUST && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set pitch adj %f\n", res);
        #endif
        set_pitch_adjust(res);
        return;
    }
    if (reg == SET_ROLL_ADJUST && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set roll adj %f\n", res);
        #endif
        set_roll_adjust(res);
        return;
    }
    if (reg == CALIBRATE_ACC && size == 1)
    {
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("calibrate acc\n");
        #endif
        schedule_acc_calibration();
        return;
    }
    if (reg == CALIBRATE_GYRO && size == 1)
    {
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("calibrate gyro\n");
        #endif
        schedule_gyro_calibration();
        return;
    }
    if (reg == CALIBRATE_MAG && size == 1)
    {
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("calibrate mag\n");
        #endif
        schedule_mag_calibration();
        return;
    }
    if (reg == CALIBRATE_ESC && size == 1)
    {
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("calibrate esc\n");
        #endif
        schedule_esc_calibration();
        return;
    }
    if (reg == SET_ACC_CALIBRATION && size == 12)
    {
        vec3 result;
        result[0] = get_float_from_net(buf_with_val);
        result[1] = get_float_from_net(buf_with_val + 4);
        result[2] = get_float_from_net(buf_with_val + 8);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set acc calib %f %f %f\n", result[0], result[1], result[2]);
        #endif
        set_acc_calibration(result);
        return;
    }
    if (reg == SET_GYRO_CALIBRATION && size == 12)
    {
        vec3 result;
        result[0] = get_float_from_net(buf_with_val);
        result[1] = get_float_from_net(buf_with_val + 4);
        result[2] = get_float_from_net(buf_with_val + 8);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set gyro calib %f %f %f\n", result[0], result[1], result[2]);
        #endif
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
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set mag calib mid %f %f %f range %f %f %f\n", mid[0], mid[1], mid[2], range[0], range[1], range[2]);
        #endif
        set_mag_calibration(mid, range);
        return;
    }
    if (reg == SET_ACC_FILTERING && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set acc filtering %f\n", res);
        #endif
        set_acc_filtering(res);
        return;
    }
    if (reg == RESET_LANDING_FLAG && size == 1)
    {
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("reset landing flag\n");
        #endif
        set_landing_flag(0);
        set_desired_height(0.f);
        return;
    }
    if (reg == SWITCH_TO_RELATIVE_ACCELERATION && size == 1)
    {
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("switch to relative acceleartion\n");
        #endif
        set_desired_relative_acceleration(0.f);
        set_use_relative_acceleration(1);
        return;
    }
    if (reg == SET_RELATIVE_ACCELERATION && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set relative acceleration %f\n", res);
        #endif
        set_desired_relative_acceleration(res);
        return;
    }
    if (reg == SET_US_HEIGHT_FILTERING && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set us height filtering %f\n", res);
        #endif
        set_us_height_filtering(res);
        return;
    }
    if (reg == SET_US_HEIGHT_DER_FILTERING && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set us height der filtering %f\n", res);
        #endif
        set_us_height_der_filtering(res);
        return;
    }
    if (reg == SET_PITCH_I_LIMIT && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set pitch i limit %f\n", res);
        #endif
        set_pitch_i_limit(res);
        return;
    }
    if (reg == SET_ROLL_I_LIMIT && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set roll i limit %f\n", res);
        #endif
        set_roll_i_limit(res);
        return;
    }
    if (reg == SET_YAW_I_LIMIT && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set yaw i limit %f\n", res);
        #endif
        set_yaw_i_limit(res);
        return;
    }
    if (reg == SET_HEIGHT_I_LIMIT && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set height i limit %f\n", res);
        #endif
        set_height_i_limit(res);
        return;
    }
    if (reg == SET_MOTOR_CURVE_A && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set motor curve a %f\n", res);
        #endif
        set_motor_curve_a(res);
        return;
    }
    if (reg == SET_MOTOR_CURVE_B && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set motor curve b %f\n", res);
        #endif
        set_motor_curve_b(res);
        return;
    }
    if (reg == SET_VOLTAGE_DROP_CURVE_A && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set voltage drop curve a %f\n", res);
        #endif
        set_voltage_drop_curve_a(res);
        return;
    }
    if (reg == SET_VOLTAGE_DROP_CURVE_B && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set voltage drop curve b %f\n", res);
        #endif
        set_voltage_drop_curve_b(res);
        return;
    }
    if (reg == SET_POWER_LOSS_CURVE_A && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set power loss curve a %f\n", res);
        #endif
        set_power_loss_curve_a(res);
        return;
    }
    if (reg == SET_POWER_LOSS_CURVE_B && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set power loss curve b %f\n", res);
        #endif
        set_power_loss_curve_b(res);
        return;
    }
    if (reg == SET_HEIGHT_NEGATIVE_INT_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set height negative int coef %f\n", res);
        #endif
        set_height_negative_int_coef(res);
        return;
    }
    if (reg == SET_POSITION_PROP_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set position prop coef %f\n", res);
        #endif
        set_position_prop_coef(res);
        return;
    }
    if (reg == SET_POSITION_DER_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set position der coef %f\n", res);
        #endif
        set_position_der_coef(res);
        return;
    }
    if (reg == SET_POSITION_INT_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set position int coef %f\n", res);
        #endif
        set_position_int_coef(res);
        return;
    }
    if (reg == SET_POSITION_I_LIMIT && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set position i limit %f\n", res);
        #endif
        set_position_i_limit(res);
        return;
    }
    if (reg == SET_BAR_HEIGHT_PROP_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set bar height prop coef %f\n", res);
        #endif
        set_bar_height_prop_coef(res);
        return;
    }
    if (reg == SET_BAR_HEIGHT_DER_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set bar height der coef %f\n", res);
        #endif
        set_bar_height_der_coef(res);
        return;
    }
    if (reg == SET_BAR_HEIGHT_INT_COEF && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set bar height int coef %f\n", res);
        #endif
        set_bar_height_int_coef(res);
        return;
    }
    if (reg == SET_BAR_HEIGHT_FILTERING && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set bar height filtering %f\n", res);
        #endif
        set_bar_height_filtering(res);
        return;
    }
    if (reg == SET_BAR_HEIGHT_DER_FILTERING && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set bar height der filtering %f\n", res);
        #endif
        set_bar_height_der_filtering(res);
        return;
    }
    if (reg == SET_POSITION_FILTERING && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set position filtering %f\n", res);
        #endif
        set_position_filtering(res);
        return;
    }
    if (reg == SET_POSITION_DER_FILTERING && size == 4)
    {
        float res = get_float_from_net(buf_with_val);
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set position der filtering %f\n", res);
        #endif
        set_position_der_filtering(res);
        return;
    }
    if (reg == SET_HOLD_MODE && size == 1)
    {
        #ifdef PRINT_ALL_SET_COMMANDS
        printf("set hold mode %d\n", buf_with_val[0]);
        #endif
        set_hold_mode(buf_with_val[0]);
        return;
    }
    #ifdef PRINT_ALL_SET_COMMANDS
    printf("unknown set command\n");
    #endif
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
    WORD_ALIGNED_ATTR uint8_t sendbuf[32];
    WORD_ALIGNED_ATTR uint8_t recvbuf[32];

    uint8_t command_type = 0; // 0 - wait for command, 1 - write, 2 - send write ack, 3 - read
    uint8_t reg = 0;
    uint8_t ack_byte = 0;

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
            t.length = 8;
            sendbuf[0] = ack_byte;
        }
        if (command_type == 3)
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
                command_type = 3;
                reg = recvbuf[0] & 0b01111111;
                continue;
            }
            command_type = 1;
            reg = recvbuf[0];
            continue;
        }
        else if (t.trans_len >= 8 && command_type == 1) 
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
                ack_byte = ~crc;
            }
            else {
                #ifdef PRINT_ALL_SET_COMMANDS
                printf("crc fail\n");
                #endif
                ack_byte = 0;
            }
            command_type = 2;
        }
        else
        {
            if (t.trans_len < 8 && command_type == 1)
            {
                #ifdef PRINT_ALL_SET_COMMANDS
                printf("receive trans length < 8 bit\n");
                #endif
            }
            command_type = 0;
            ack_byte = 0;
        }
    }
}

void communication_setup()
{
    xTaskCreate(communication_task, "communication_task", 5000, NULL, 1, &communicationTaskHandle);
}