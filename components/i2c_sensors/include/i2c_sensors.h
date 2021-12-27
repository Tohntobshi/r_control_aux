#include "cglm/cglm.h"

/*
    set_gyro_calibration,
    set_mag_calibration,
    get_gyro_calibration,
    get_mag_calibration
    can be called from any task

    other functions are expected to be managed by one task
*/

void i2c_sensors_setup();
void set_acc_filtering_mode(uint8_t mode); // from 1 to 6
void set_gyro_filtering_mode(uint8_t mode); // from 1 to 6
void set_gyro_calibration(vec3 value);
void set_mag_calibration(vec3 mid_vals, vec3 range);
void set_acc_calibration(vec3 value);

void get_acc_calibrated_data(vec3 dest);
void get_gyro_calibrated_data(vec3 dest);
void get_mag_normalized_data(vec3 dest);
float get_bar_data();

void calibrate_gyro();
void get_gyro_calibration(vec3 dest);

void calibrate_mag();
void get_mag_calibration(vec3 mid_vals_dest, vec3 range_dest);

void calibrate_acc();
void get_acc_calibration(vec3 dest);

