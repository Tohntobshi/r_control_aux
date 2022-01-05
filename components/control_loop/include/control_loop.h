#include "cglm/cglm.h"

void control_loop_setup();

void set_move_vector(float x, float y);
void set_desired_direction(float val);
void set_desired_height(float val);
void set_base_acceleration(float val);
void set_pitch_prop_coef(float val);
void set_pitch_der_coef(float val);
void set_pitch_int_coef(float val);
void set_roll_prop_coef(float val);
void set_roll_der_coef(float val);
void set_roll_int_coef(float val);
void set_yaw_prop_coef(float val);
void set_yaw_der_coef(float val);
void set_yaw_int_coef(float val);
void set_height_prop_coef(float val);
void set_height_der_coef(float val);
void set_height_int_coef(float val);
void set_height_negative_int_coef(float val);
void set_acc_trust(float val);
void set_mag_trust(float val);
void schedule_set_acc_filtering_mode(uint8_t val);
void schedule_set_gyro_filtering_mode(uint8_t val);
void reset_turn_off_trigger();
void set_turn_off_incline_angle(float val);
void set_pitch_adjust(float val);
void set_roll_adjust(float val);
void schedule_acc_calibration();
void schedule_gyro_calibration();
void schedule_mag_calibration();
void schedule_esc_calibration();
void set_gyro_calibration(vec3 value);
void set_mag_calibration(vec3 mid_vals, vec3 range);
void set_acc_filtering(float val);
void set_use_relative_acceleration(uint8_t val);
void set_desired_relative_acceleration(float val);
void set_landing_flag(uint8_t val);
void set_us_height_filtering(float val);
void set_us_height_der_filtering(float val);
void set_pitch_i_limit(float val);
void set_roll_i_limit(float val);
void set_yaw_i_limit(float val);
void set_height_i_limit(float val);
void set_acc_calibration(vec3 value);
void set_motor_curve_a(float val);
void set_motor_curve_b(float val);
void set_voltage_drop_curve_a(float val);
void set_voltage_drop_curve_b(float val);
void set_power_loss_curve_a(float val);
void set_power_loss_curve_b(float val);
void set_position_prop_coef(float val);
void set_position_der_coef(float val);
void set_position_int_coef(float val);
void set_position_i_limit(float val);
void set_bar_height_prop_coef(float val);
void set_bar_height_der_coef(float val);
void set_bar_height_int_coef(float val);
void set_bar_height_filtering(float val);
void set_bar_height_der_filtering(float val);
void set_position_filtering(float val);
void set_position_der_filtering(float val);
void set_hold_mode(uint8_t val);


float get_current_pitch_err();
float get_current_pitch_err_der();
float get_pitch_err_int();

float get_current_roll_err();
float get_current_roll_err_der();
float get_roll_err_int();

float get_current_height_err();
float get_current_height_err_der();
float get_height_err_int();

float get_current_yaw_err();
float get_current_yaw_err_der();
float get_yaw_err_int();

uint16_t get_fl_motor_val();
uint16_t get_fr_motor_val();
uint16_t get_bl_motor_val();
uint16_t get_br_motor_val();
float get_loop_freq();

void get_acc_calibration(vec3 dest);
void get_gyro_calibration(vec3 dest);
void get_mag_calibration(vec3 mid_vals_dest, vec3 range_dest);

uint8_t get_landing_flag();

float get_base_voltage();

float get_x_position_err();
float get_y_position_err();
float get_x_position_err_der();
float get_y_position_err_der();
float get_x_position_err_int();
float get_y_position_err_int();

enum hold_mode {
    HOLD_OFF,
    HOLD_DIRECTION,
    HOLD_POSITION
};