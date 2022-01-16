#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "i2c_sensors.h"
#include "VL53L1X_api.h"
#include "bmp280.h"
#include "utils.h"


#define SDA_PIN 32
#define SCL_PIN 33

#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C
#define VL53L1X_ADDRESS 0x29

#define ACCEL_XOUT_H 0x3B // Accel data first register
#define GYRO_XOUT_H 0x43  // Gyro data first register
#define USER_CTRL 0x6A	  // MPU9250 config
#define INT_PIN_CFG 0x37  // MPU9250 config
#define CONFIG 0x1A	// MPU9250 config
#define ACCEL_CONFIG_2 0x1D // MPU9250 config

#define AK8963_CNTL 0x0A   // Mag config
#define AK8963_XOUT_L 0x03 // Mag data first register
#define AK8963_ASAX 0x10   // Mag adj vals first register

static float aRes = 2.0 / 32768.0;
static float gRes = 250.0 / 32768.0;

static struct bmp280_dev bmp;

static vec3 mag_adjustment_vals;
static vec3 mag_range;
static vec3 mag_mid_vals;
static vec3 gyro_calibration;
static vec3 acc_calibration;

static SemaphoreHandle_t accCalibMtx = NULL;
static SemaphoreHandle_t gyroCalibMtx = NULL;
static SemaphoreHandle_t magCalibMtx = NULL;

static int8_t write_bytes(uint8_t address, uint8_t reg, uint8_t * data, uint16_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    i2c_master_write(cmd, data, size, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return 0;
}

static int8_t write_bytes_16bit_reg(uint8_t address, uint16_t reg, uint8_t * data, uint16_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    set_short_to_net(reg, (uint8_t *)&reg);
    i2c_master_write(cmd, (uint8_t *)&reg, 2, I2C_MASTER_ACK);
    i2c_master_write(cmd, data, size, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return 0;
}

static int8_t read_bytes(uint8_t address, uint8_t reg, uint8_t * dest, uint16_t size)
{
    i2c_cmd_handle_t wcmd = i2c_cmd_link_create();
    i2c_master_start(wcmd);
    i2c_master_write_byte(wcmd, (address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(wcmd, reg, I2C_MASTER_ACK);
    i2c_master_stop(wcmd);
    i2c_master_cmd_begin(I2C_NUM_0, wcmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(wcmd);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, dest, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return 0;
}

static int8_t read_bytes_16bit_reg(uint8_t address, uint16_t reg, uint8_t * dest, uint16_t size)
{
    i2c_cmd_handle_t wcmd = i2c_cmd_link_create();
    i2c_master_start(wcmd);
    i2c_master_write_byte(wcmd, (address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    set_short_to_net(reg, (uint8_t *)&reg);
    i2c_master_write(wcmd, (uint8_t *)&reg, 2, I2C_MASTER_ACK);
    i2c_master_stop(wcmd);
    i2c_master_cmd_begin(I2C_NUM_0, wcmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(wcmd);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read(cmd, dest, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return 0;
}

static void get_acc_data(vec3 dest)
{
	uint8_t rawData[6];
	read_bytes(MPU9250_ADDRESS, ACCEL_XOUT_H, rawData, 6);
	int16_t rawX = ((int16_t)rawData[0] << 8) | rawData[1];
	int16_t rawY = ((int16_t)rawData[2] << 8) | rawData[3];
	int16_t rawZ = ((int16_t)rawData[4] << 8) | rawData[5];

	dest[0] = (float)rawX * aRes;
	dest[1] = (float)rawY * aRes;
	dest[2] = (float)rawZ * aRes;
}

static void get_gyro_data(vec3 dest)
{
    uint8_t rawData[6];
	read_bytes(MPU9250_ADDRESS, GYRO_XOUT_H, rawData, 6);
	int16_t rawX = ((int16_t)rawData[0] << 8) | rawData[1];
	int16_t rawY = ((int16_t)rawData[2] << 8) | rawData[3];
	int16_t rawZ = ((int16_t)rawData[4] << 8) | rawData[5];

	dest[0] = (float)rawX * gRes;
	dest[1] = (float)rawY * gRes;
	dest[2] = (float)rawZ * gRes;
}

static void get_mag_data(vec3 dest)
{
    while (1)
	{
		uint8_t rawData[7];
        read_bytes(AK8963_ADDRESS, AK8963_XOUT_L, rawData, 7);
		if ((rawData[6] & 0x08))
		{
			// data corrupted, try again
			continue;
		}
		int16_t rawX = ((int16_t)rawData[1] << 8) | rawData[0];
		int16_t rawY = ((int16_t)rawData[3] << 8) | rawData[2];
		int16_t rawZ = ((int16_t)rawData[5] << 8) | rawData[4];
		dest[0] = (float)rawX * mag_adjustment_vals[0];
		dest[1] = (float)rawY * mag_adjustment_vals[1];
		dest[2] = (float)rawZ * mag_adjustment_vals[2];
        return;
	}
}

float get_bar_data()
{
	struct bmp280_uncomp_data ucomp_data;
	double pres;
	bmp280_get_uncomp_data(&ucomp_data, &bmp);
	bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
	return pres;
}

void calibrate_acc()
{
    vec3 values = { 0.f, 0.f, 0.f };
    vec3 zero = { 0.f, 0.f, 1.f };
	for (int i = 0; i < 10000; i++) {
		vec3 acc;
        get_acc_data(acc);
        glm_vec3_sub(acc, zero, acc);
        glm_vec3_add(acc, values, values);
	}
    xSemaphoreTake(accCalibMtx, portMAX_DELAY);
    glm_vec3_divs(values, 10000.f, acc_calibration);
    xSemaphoreGive(accCalibMtx);
}

void set_acc_calibration(vec3 value)
{
    xSemaphoreTake(accCalibMtx, portMAX_DELAY);
    glm_vec3_copy(value, acc_calibration);
    xSemaphoreGive(accCalibMtx);
}
void get_acc_calibration(vec3 dest)
{
    xSemaphoreTake(accCalibMtx, portMAX_DELAY);
    glm_vec3_copy(acc_calibration, dest);
    xSemaphoreGive(accCalibMtx);
}

void get_acc_calibrated_data(vec3 dest)
{
    get_acc_data(dest);
    xSemaphoreTake(accCalibMtx, portMAX_DELAY);
    glm_vec3_sub(dest, acc_calibration, dest);
    xSemaphoreGive(accCalibMtx);
}

void get_gyro_calibrated_data(vec3 dest)
{
    get_gyro_data(dest);
    xSemaphoreTake(gyroCalibMtx, portMAX_DELAY);
    glm_vec3_sub(dest, gyro_calibration, dest);
    xSemaphoreGive(gyroCalibMtx);
}

void calibrate_gyro()
{
    vec3 values = { 0.f, 0.f, 0.f };
	for (int i = 0; i < 10000; i++) {
		vec3 gyro;
        get_gyro_data(gyro);
        glm_vec3_add(gyro, values, values);
	}
    xSemaphoreTake(gyroCalibMtx, portMAX_DELAY);
    glm_vec3_divs(values, 10000.f, gyro_calibration);
    xSemaphoreGive(gyroCalibMtx);
}


void set_gyro_calibration(vec3 value)
{
    xSemaphoreTake(gyroCalibMtx, portMAX_DELAY);
    glm_vec3_copy(value, gyro_calibration);
    xSemaphoreGive(gyroCalibMtx);
}

void get_gyro_calibration(vec3 dest)
{
    xSemaphoreTake(gyroCalibMtx, portMAX_DELAY);
    glm_vec3_copy(gyro_calibration, dest);
    xSemaphoreGive(gyroCalibMtx);
}


void calibrate_mag()
{
    vec3 mag_min_vals;
    vec3 mag_max_vals;
    get_mag_data(mag_min_vals);
    glm_vec3_copy(mag_min_vals, mag_max_vals);
    for (int i = 0; i < 30000; i++) {
		vec3 res;
        get_mag_data(res);
		if (res[0] < mag_min_vals[0])
			mag_min_vals[0] = res[0];
		if (res[0] > mag_max_vals[0])
			mag_max_vals[0] = res[0];
		if (res[1] < mag_min_vals[1])
			mag_min_vals[1] = res[1];
		if (res[1] > mag_max_vals[1])
			mag_max_vals[1] = res[1];
		if (res[2] < mag_min_vals[2])
			mag_min_vals[2] = res[2];
		if (res[2] > mag_max_vals[2])
			mag_max_vals[2] = res[2];
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
    xSemaphoreTake(magCalibMtx, portMAX_DELAY);
    glm_vec3_sub(mag_max_vals, mag_min_vals, mag_range);
    glm_vec3_divs(mag_range, 2.f, mag_range);
    glm_vec3_add(mag_min_vals, mag_range, mag_mid_vals);
    xSemaphoreGive(magCalibMtx);
}

void get_mag_calibration(vec3 mid_vals_dest, vec3 range_dest)
{
    xSemaphoreTake(magCalibMtx, portMAX_DELAY);
    glm_vec3_copy(mag_mid_vals, mid_vals_dest);
    glm_vec3_copy(mag_range, range_dest);
    xSemaphoreGive(magCalibMtx);
}

void set_mag_calibration(vec3 mid_vals, vec3 range)
{
    xSemaphoreTake(magCalibMtx, portMAX_DELAY);
    glm_vec3_copy(mid_vals, mag_mid_vals);
    glm_vec3_copy(range, mag_range);
    xSemaphoreGive(magCalibMtx);
}

void get_mag_normalized_data(vec3 dest)
{
    get_mag_data(dest);
    xSemaphoreTake(magCalibMtx, portMAX_DELAY);
    dest[0] = (dest[0] - mag_mid_vals[0]) / mag_range[0];
    dest[1] = (dest[1] - mag_mid_vals[1]) / mag_range[1];
    dest[2] = (dest[2] - mag_mid_vals[2]) / mag_range[2];
    xSemaphoreGive(magCalibMtx);
    glm_vec3_normalize(dest);
}

void set_acc_filtering_mode(uint8_t mode)
{
    write_bytes(MPU9250_ADDRESS, ACCEL_CONFIG_2, &mode, 1);
}

void set_gyro_filtering_mode(uint8_t mode)
{
    write_bytes(MPU9250_ADDRESS, CONFIG, &mode, 1);
}

static int8_t VL53L1_WrByte(uint16_t index, uint8_t data) {
	return write_bytes_16bit_reg(VL53L1X_ADDRESS, index, &data, 1);
}

static int8_t VL53L1_WrWord(uint16_t index, uint16_t data) {
    uint8_t data_to_send[2];
    set_short_to_net(data, data_to_send);
	return write_bytes_16bit_reg(VL53L1X_ADDRESS, index, data_to_send, 2);
}

static int8_t VL53L1_WrDWord(uint16_t index, uint32_t data) {
    uint8_t data_to_send[4];
    set_int_to_net(data, data_to_send);
	return write_bytes_16bit_reg(VL53L1X_ADDRESS, index, data_to_send, 4);
}

static int8_t VL53L1_RdByte(uint16_t index, uint8_t *data) {
	return read_bytes_16bit_reg(VL53L1X_ADDRESS, index, data, 1);
}

static int8_t VL53L1_RdWord(uint16_t index, uint16_t *data) {
    uint8_t data_to_get[2];
	read_bytes_16bit_reg(VL53L1X_ADDRESS, index, data_to_get, 2);
    *data = get_short_from_net(data_to_get);
    return 0;
}

static int8_t VL53L1_ReadMulti(uint16_t index, uint8_t *pdata, uint32_t count){
	return read_bytes_16bit_reg(VL53L1X_ADDRESS, index, pdata, count);
}

float get_bottom_distance()
{
    uint16_t distance;
    VL53L1X_GetDistance(&distance);
    return distance / 1000.f;
}

void i2c_sensors_setup()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master = {
            .clk_speed = 400000
        },
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_LOWMED);


    uint8_t int_pin_cfg;
    read_bytes(MPU9250_ADDRESS, INT_PIN_CFG, &int_pin_cfg, 1);
    int_pin_cfg |= 0b00000010; // set pass though mode for mpu9250 in order to have direct access to magnetometer
    write_bytes(MPU9250_ADDRESS, INT_PIN_CFG, &int_pin_cfg, 1);

    uint8_t mag_mode = 0b00000000; // set power down mode for magnetometer
    write_bytes(AK8963_ADDRESS, AK8963_CNTL, &mag_mode, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    mag_mode = 0b00001111; // set rom mode for magnetometer
    write_bytes(AK8963_ADDRESS, AK8963_CNTL, &mag_mode, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uint8_t raw_mag_adj_vals[3];
    read_bytes(AK8963_ADDRESS, AK8963_ASAX, raw_mag_adj_vals, 3);
    mag_adjustment_vals[0] = ((float)(raw_mag_adj_vals[0] - 128) * 0.5f) / 128.f + 1.f;
	mag_adjustment_vals[1] = ((float)(raw_mag_adj_vals[1] - 128) * 0.5f) / 128.f + 1.f;
	mag_adjustment_vals[2] = ((float)(raw_mag_adj_vals[2] - 128) * 0.5f) / 128.f + 1.f;
    mag_mode = 0b00000000; // set power down mode for magnetometer
    write_bytes(AK8963_ADDRESS, AK8963_CNTL, &mag_mode, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    mag_mode = 0b00010110; // set 16bit continious measurement mode for magnetometer
    write_bytes(AK8963_ADDRESS, AK8963_CNTL, &mag_mode, 1);


    bmp.delay_ms = vTaskDelay;
	bmp.dev_id = BMP280_I2C_ADDR_PRIM;
	bmp.read = read_bytes;
	bmp.write = write_bytes;
	bmp.intf = BMP280_I2C_INTF;
	bmp280_init(&bmp);
	struct bmp280_config bmpConf;
	bmp280_get_config(&bmpConf, &bmp);
	bmpConf.filter = BMP280_FILTER_COEFF_2;
	bmpConf.os_pres = BMP280_OS_4X;
	bmpConf.odr = BMP280_ODR_62_5_MS;
	bmp280_set_config(&bmpConf, &bmp);
	bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    
    VL53L1X_ReadWrite_Functions func;
    func.rdByte = VL53L1_RdByte;
    func.rdWord = VL53L1_RdWord;
    func.rdMulti = VL53L1_ReadMulti;
    func.wrByte = VL53L1_WrByte;
    func.wrWord = VL53L1_WrWord;
    func.wrDWord = VL53L1_WrDWord;
    VL53L1X_SetReadWriteFunctions(func);
    
    uint8_t sensor_state = 0;
    while(sensor_state==0){
		VL53L1X_BootState(&sensor_state);
	    vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    VL53L1X_SensorInit();
    VL53L1X_SetInterMeasurementInMs(33);
    VL53L1X_SetTimingBudgetInMs(33);
    VL53L1X_StartRanging();

    accCalibMtx = xSemaphoreCreateMutex();
    gyroCalibMtx = xSemaphoreCreateMutex();
    magCalibMtx = xSemaphoreCreateMutex();
}
