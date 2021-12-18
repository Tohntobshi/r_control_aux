#include <stdio.h>
#include <math.h>
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "control_loop.h"
#include "i2c_sensors.h"
#include "motor_control.h"
#include "us_sensor.h"

#define MIN_VAL 1000
#define MAX_VAL 2000

static TaskHandle_t controlLoopTaskHandle;

// -------- in values --------

// desired values
static float desiredPitchIn = 0.f;
static float desiredRollIn = 0.f;
static float desiredDirectionIn = 0.f;
static float desiredHeightIn = 0.f;

// PID coefficients for pitch
static float pitchPropCoefIn = 0.f;
static float pitchDerCoefIn = 0.f;
static float pitchIntCoefIn = 0.f;

// PID coefficients for roll
static float rollPropCoefIn = 0.f;
static float rollDerCoefIn = 0.f;
static float rollIntCoefIn = 0.f;

// PID coefficients for yaw
static float yawPropCoefIn = 0.f;
static float yawDerCoefIn = 0.f;
static float yawIntCoefIn = 0.f;

// PID coefficients for height
static float heightPropCoefIn = 0.f;
static float heightDerCoefIn = 0.f;
static float heightIntCoefIn = 0.f;

// filtering
static float accTrustIn = 0.1f;
static float magTrustIn = 0.1f;
static uint8_t imuLPFModeIn = 3;

// safety
static uint8_t turnOffTrigger = 0;
static float turnOffInclineAngleIn = 30.f;

// adjustments
static float pitchAdjustIn = 0.f;
static float rollAdjustIn = 0.f;
static float baseAccelerationIn = 0.f;

// calibration
static uint8_t needCalibrateGyro = 0;
static uint8_t needCalibrateMag = 0;
static uint8_t needCalibrateEsc = 0;

// -------- out values --------

static float currentPitchErrorOut = 0.f;
static float currentRollErrorOut = 0.f;

static float pitchErrorChangeRateOut = 0.f;
static float rollErrorChangeRateOut = 0.f;

static float currentHeightErrorOut = 0.f;
static float heightErrorChangeRateOut = 0.f;

static float currentYawErrorOut = 0.f;
static float yawErrorChangeRateOut = 0.f;

static uint16_t frontLeftOut = 0;
static uint16_t frontRightOut = 0;
static uint16_t backLeftOut = 0;
static uint16_t backRightOut = 0;

static float loopFrequencyOut = 0.f;

static float pitchErrIntOut = 0.f;
static float rollErrIntOut = 0.f;
static float yawErrIntOut = 0.f;
static float heightErrIntOut = 0.f;

void set_desired_pitch_and_roll(float pitch, float roll)
{
    desiredPitchIn = pitch;
    desiredRollIn = roll;
}

void set_desired_direction(float val)
{
    desiredDirectionIn = val;
}

void set_desired_height(float val)
{
    desiredHeightIn = val;
}

void set_base_acceleration(float val)
{
    baseAccelerationIn = val;
}

void set_pitch_prop_coef(float val)
{
    pitchPropCoefIn = val;
}

void set_pitch_der_coef(float val)
{
    pitchDerCoefIn = val;
}

void set_pitch_int_coef(float val)
{
    pitchIntCoefIn = val;
}

void set_roll_prop_coef(float val)
{
    rollPropCoefIn = val;
}

void set_roll_der_coef(float val)
{
    rollDerCoefIn = val;
}

void set_roll_int_coef(float val)
{
    rollIntCoefIn = val;
}

void set_yaw_prop_coef(float val)
{
    yawPropCoefIn = val;
}

void set_yaw_der_coef(float val)
{
    yawDerCoefIn = val;
}

void set_yaw_int_coef(float val)
{
    yawIntCoefIn = val;
}

void set_height_prop_coef(float val)
{
    heightPropCoefIn = val;
}

void set_height_der_coef(float val)
{
    heightDerCoefIn = val;
}

void set_height_int_coef(float val)
{
    heightIntCoefIn = val;
}

void set_acc_trust(float val)
{
    accTrustIn = val;
}

void set_mag_trust(float val)
{
    magTrustIn = val;
}

void schedule_set_acc_gyro_filtering_mode(uint8_t val)
{
    imuLPFModeIn = val;
}

void reset_turn_off_trigger()
{
    turnOffTrigger = 0;
}

void set_turn_off_incline_angle(float val)
{
    turnOffInclineAngleIn = val;
}

void set_pitch_adjust(float val)
{
    pitchAdjustIn = val;
}

void set_roll_adjust(float val)
{
    rollAdjustIn = val;
}

void schedule_gyro_calibration()
{
    needCalibrateGyro = 1;
}

void schedule_mag_calibration()
{
    needCalibrateMag = 1;
}

void schedule_esc_calibration()
{
    needCalibrateEsc = 1;
}

float get_current_pitch_err()
{
    return currentPitchErrorOut;
}

float get_current_roll_err()
{
    return currentRollErrorOut;
}

float get_current_pitch_err_der()
{
    return pitchErrorChangeRateOut;
}

float get_current_roll_err_der()
{
    return rollErrorChangeRateOut;
}

float get_current_height_err()
{
    return currentHeightErrorOut;
}

float get_current_height_err_der()
{
    return heightErrorChangeRateOut;
}

float get_current_yaw_err()
{
    return currentYawErrorOut;
}

float get_current_yaw_err_der()
{
    return yawErrorChangeRateOut;
}

uint16_t get_fl_motor_val()
{
    return frontLeftOut;
}

uint16_t get_fr_motor_val()
{
    return frontRightOut;
}

uint16_t get_bl_motor_val()
{
    return backLeftOut;
}

uint16_t get_br_motor_val()
{
    return backRightOut;
}

float get_loop_freq()
{
    return loopFrequencyOut;
}

float get_pitch_err_int()
{
    return pitchErrIntOut;
}

float get_roll_err_int()
{
    return rollErrIntOut;
}

float get_yaw_err_int()
{
    return yawErrIntOut;
}

float get_height_err_int()
{
    return heightErrIntOut;
}

static uint8_t check_scheduled_actions()
{
    if(needCalibrateGyro)
    {
        calibrate_gyro();
        needCalibrateGyro = 0;
        return 1;
    }
    if(needCalibrateMag)
    {
        calibrate_mag();
        needCalibrateMag = 0;
        return 1;
    }
    if(needCalibrateEsc)
    {
        calibrate_esc();
        needCalibrateEsc = 0;
        return 1;
    }
    if(imuLPFModeIn)
    {
        set_acc_gyro_filtering_mode(imuLPFModeIn);
        imuLPFModeIn = 0;
        return 1;
    }
    return 0;
}

static float trim_angle_to_360(float angle) {
    if (angle < 0.f) {
        return trim_angle_to_360(angle + 360.f);
    } else if (angle >= 360.f) {
        return trim_angle_to_360(angle - 360.f);
    } else {
        return angle;
    }
}

static void pepare_angles_for_combination(float * angle1, float * angle2) {
    *angle1 = trim_angle_to_360(*angle1);
    *angle2 = trim_angle_to_360(*angle2);
    if (fabs(*angle2 - *angle1) <= 180.f) {
        return;
    } else if (*angle1 > *angle2) {
        *angle2 = *angle2 + 360.f;
        return;
    } else {
        *angle1 = *angle1 + 360.f;
        return;
    }
}

static int clamp_integer(int val, int min, int max)
{
    return val > max ? max : (val < min ? min : val);
}

static void control_loop_task(void * params)
{
    float prevPitch = 0.f;
	float prevRoll = 0.f;
	float prevYaw = 0.f;
	float prevHeight = 0.f;
	float prevHeightDer = 0.f;
	float pitchErrInt = 0.f;
	float rollErrInt = 0.f;
	float yawErrInt = 0.f;
	float heightErrInt = 0.f;
    float prevLoopFreq = 0.f;

    while(1)
    {
        // check scheduled actions
        if(check_scheduled_actions())
        {
            timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
            continue;
        }

        // copy all user input values
        float desiredPitch = desiredPitchIn;
		float desiredRoll = desiredRollIn;
		float pitchAdjust = pitchAdjustIn;
		float rollAdjust = rollAdjustIn;
		float desiredDirection = -desiredDirectionIn;
		float desiredHeight = desiredHeightIn;
		float accTrust = accTrustIn;
		float magTrust = magTrustIn;
		float turnOffInclineAngle = turnOffInclineAngleIn;
		float pitchPropCoef = pitchPropCoefIn;
		float pitchDerCoef = pitchDerCoefIn;
		float pitchIntCoef = pitchIntCoefIn;
		float rollPropCoef = rollPropCoefIn;
		float rollDerCoef = rollDerCoefIn;
		float rollIntCoef = rollIntCoefIn;
		float yawPropCoef = yawPropCoefIn;
		float yawDerCoef = yawDerCoefIn;
		float yawIntCoef = yawIntCoefIn;
		float heightPropCoef = heightPropCoefIn;
		float heightDerCoef = heightDerCoefIn;
		float heightIntCoef = heightIntCoefIn;
		float baseAcceleration = baseAccelerationIn;

        // get all sensor and time data
        vec3 acc_data;
        get_acc_data(acc_data);
        vec3 gyro_data;
        get_gyro_calibrated_data(gyro_data);
        vec3 mag_data;
        get_mag_normalized_data(mag_data);
        float bar_data = get_bar_data();
        float usonic_distance = get_current_us_distance();
        uint64_t u_seconds_elapsed;
        timer_get_counter_value(TIMER_GROUP_0, TIMER_1, &u_seconds_elapsed);
        timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);

        // computing values
        float seconds_elapsed = u_seconds_elapsed / 1000000.f;

        float accPitch = glm_deg(atan2(acc_data[0], acc_data[2]));
        vec2 tmp1 = { acc_data[2], acc_data[0] };
		float accRoll = -glm_deg(atan2(acc_data[1], glm_vec2_norm(tmp1)));

        float gyroPitch = prevPitch - gyro_data[1] * seconds_elapsed;
		float gyroRoll = prevRoll - gyro_data[0] * seconds_elapsed;
        
		float gyroYaw = prevYaw + gyro_data[2] * seconds_elapsed;
		float gyroPitchTmp = gyroPitch;
		float gyroRollTmp = gyroRoll;
        gyroPitch -= gyroRollTmp * sin(glm_rad(gyro_data[2] * seconds_elapsed));
		gyroRoll += gyroPitchTmp * sin(glm_rad(gyro_data[2] * seconds_elapsed));
		gyroYaw += gyro_data[0] * seconds_elapsed * sin(glm_rad(gyroPitchTmp));
		gyroYaw -= gyro_data[1] * seconds_elapsed * sin(glm_rad(gyroRollTmp));

        float currentPitch = accPitch * accTrust + (1.f - accTrust) * gyroPitch;
		float currentRoll = accRoll * accTrust + (1.f - accTrust) * gyroRoll;

        vec3 xAxis = { 1.f, 0.f, 0.f };
        vec3 yAxis = { 0.f, 1.f, 0.f };
        glm_vec3_rotate(mag_data, glm_rad(-currentPitch), xAxis);
        glm_vec3_rotate(mag_data, glm_rad(-currentRoll), yAxis);
        float magYaw = glm_deg(atan2(mag_data[1], mag_data[0])) - 90.f;

        pepare_angles_for_combination(&magYaw, &gyroYaw);
        float currentYaw = trim_angle_to_360(magYaw * magTrust + (1.f - magTrust) * gyroYaw);
        
        if (isnan(currentYaw))
        {
            currentYaw = 0.f;
            yawErrInt = 0.f;
            
        }
        prevPitch = currentPitch;
		prevRoll = currentRoll;
        prevYaw = currentYaw;

        float currentPitchError = desiredPitch - currentPitch + pitchAdjust;
		float currentRollError = desiredRoll - currentRoll + rollAdjust;
        pepare_angles_for_combination(&desiredDirection, &currentYaw);
		float currentYawError = desiredDirection - currentYaw;

        float pitchErrorChangeRate = gyro_data[1];
		float rollErrorChangeRate = gyro_data[0];
		float yawErrorChangeRate = -gyro_data[2];
		
		pitchErrInt += currentPitchError * seconds_elapsed;
		rollErrInt += currentRollError * seconds_elapsed;
		yawErrInt += currentYawError * seconds_elapsed;

        float currentHeight = 0.f;
		float currentHeightError = 0.f;
		if (fabs(currentPitch) > 30.f || fabs(currentRoll) > 30.f)
		{
			currentHeight = prevHeight;
			currentHeightError = desiredHeight - currentHeight;
		}
		else
		{
			currentHeight = usonic_distance * sqrt(1.f / (pow(tan(glm_rad(currentRoll)), 2) + pow(tan(glm_rad(currentPitch)), 2) + 1)) * 0.2f + prevHeight * 0.8f;
			currentHeightError = desiredHeight - currentHeight;
			heightErrInt += currentHeightError * seconds_elapsed;
		}
		float heightDer = ((currentHeight - prevHeight) / seconds_elapsed) * 0.2f + prevHeightDer * 0.8f;
        float heightErrorChangeRate = -heightDer;
		prevHeight = currentHeight;
		prevHeightDer = heightDer;

        if (currentHeight < 0.1f) {
			currentYawError = 0.f;
			yawErrInt = 0.f;
		}
        if (fabs(currentPitch) > turnOffInclineAngle || fabs(currentRoll) > turnOffInclineAngle) {
			turnOffTrigger = 1;
		}

        int pitchMotorAdjust = currentPitchError * pitchPropCoef + pitchErrorChangeRate * pitchDerCoef + pitchErrInt * pitchIntCoef;
		int rollMotorAdjust = currentRollError * rollPropCoef + rollErrorChangeRate * rollDerCoef + rollErrInt * rollIntCoef;
		int yawMotorAdjust = currentYawError * yawPropCoef + yawErrorChangeRate * yawDerCoef + yawErrInt * yawIntCoef;
		int heightMotorAdjust = (currentHeightError * heightPropCoef + heightErrorChangeRate * heightDerCoef + heightErrInt * heightIntCoef) * 1000;
        

		int baseMotorVal = MIN_VAL + (MAX_VAL - MIN_VAL) * baseAcceleration;

		int frontLeft = clamp_integer(baseMotorVal + heightMotorAdjust + pitchMotorAdjust - rollMotorAdjust + yawMotorAdjust, MIN_VAL, MAX_VAL);
		int frontRight = clamp_integer(baseMotorVal + heightMotorAdjust + pitchMotorAdjust + rollMotorAdjust - yawMotorAdjust, MIN_VAL, MAX_VAL);
		int backLeft = clamp_integer(baseMotorVal + heightMotorAdjust - pitchMotorAdjust - rollMotorAdjust - yawMotorAdjust, MIN_VAL, MAX_VAL);
		int backRight = clamp_integer(baseMotorVal + heightMotorAdjust - pitchMotorAdjust + rollMotorAdjust + yawMotorAdjust, MIN_VAL, MAX_VAL);

		if (desiredHeight < 0.05f || turnOffTrigger)
		{
			frontLeft = MIN_VAL;
			frontRight = MIN_VAL;
			backLeft = MIN_VAL;
			backRight = MIN_VAL;
			pitchErrInt = 0.f;
			rollErrInt = 0.f;
			heightErrInt = 0.f;
			yawErrInt = 0.f;

		}
		set_motor_vals(frontLeft, frontRight, backLeft, backRight);
        
        float currentLoopFreq = (1 / seconds_elapsed) * 0.5f + prevLoopFreq * 0.5;
        prevLoopFreq = currentLoopFreq;

        // writing info output
        
        currentPitchErrorOut = currentPitchError;
        currentRollErrorOut = currentRollError;

        pitchErrorChangeRateOut = pitchErrorChangeRate;
        rollErrorChangeRateOut = rollErrorChangeRate;

        currentHeightErrorOut = currentHeightError;
        heightErrorChangeRateOut = heightErrorChangeRate;

        currentYawErrorOut = currentYawError;
        yawErrorChangeRateOut = yawErrorChangeRate;

        frontLeftOut = frontLeft;
        frontRightOut = frontRight;
        backLeftOut = backLeft;
        backRightOut = backRight;

        loopFrequencyOut = currentLoopFreq;

        pitchErrIntOut = pitchErrInt;
        rollErrIntOut = rollErrInt;
        yawErrIntOut = yawErrInt;
        heightErrIntOut = heightErrInt;
    }
}

void control_loop_setup()
{
    motor_control_setup();
    i2c_sensors_setup();
    ultrasonic_setup();
    timer_config_t t_conf = {
        .divider = 80,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_START,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    timer_init(TIMER_GROUP_0, TIMER_1, &t_conf);
    xTaskCreate(control_loop_task, "control_loop_task", 50000, NULL, 1, &controlLoopTaskHandle);
}