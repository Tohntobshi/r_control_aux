#include "cglm/cglm.h"

void i2c_sensors_setup();
void set_acc_gyro_filtering_mode(uint8_t mode); // from 1 to 6
void set_gyro_calibration(vec3 value);
void set_mag_calibration(vec3 mid_vals, vec3 range);

void get_acc_data(vec3 dest);
void get_gyro_calibrated_data(vec3 dest);
void get_mag_normalized_data(vec3 dest);
float get_bar_data();

void calibrate_gyro();
void get_gyro_calibration(vec3 dest);

void calibrate_mag();
void get_mag_calibration(vec3 mid_vals_dest, vec3 range_dest);