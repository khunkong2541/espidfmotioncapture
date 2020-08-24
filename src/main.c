#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "sdkconfig.h"

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS1 0x69 // I2C address of MPU6050
#define I2C_ADDRESS2 0x68 // I2C address of MPU6050
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B

/*
 * The following registers contain the primary data we are interested in
 * 0x3B MPU6050_ACCEL_XOUT_H
 * 0x3C MPU6050_ACCEL_XOUT_L
 * 0x3D MPU6050_ACCEL_YOUT_H
 * 0x3E MPU6050_ACCEL_YOUT_L
 * 0x3F MPU6050_ACCEL_ZOUT_H
 * 0x50 MPU6050_ACCEL_ZOUT_L
 * 0x41 MPU6050_TEMP_OUT_H
 * 0x42 MPU6050_TEMP_OUT_L
 * 0x43 MPU6050_GYRO_XOUT_H
 * 0x44 MPU6050_GYRO_XOUT_L
 * 0x45 MPU6050_GYRO_YOUT_H
 * 0x46 MPU6050_GYRO_YOUT_L
 * 0x47 MPU6050_GYRO_ZOUT_H
 * 0x48 MPU6050_GYRO_ZOUT_L
 */

static char tag[] = "mpu6050";


#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

void app_main(void *ignore) {
	ESP_LOGD(tag, ">> mpu6050");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = PIN_SDA;
	conf.scl_io_num = PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	i2c_cmd_handle_t cmd;
	vTaskDelay(200/portTICK_PERIOD_MS);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS1 << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS1 << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1);
	i2c_master_write_byte(cmd, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

//2

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS2 << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS2 << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1);
	i2c_master_write_byte(cmd, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));

	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);


	uint8_t data1[14],data2[14];
	uint64_t ti1,ti2;
	short accel_x1,accel_x2;
	short accel_y1,accel_y2;
	short accel_z1,accel_z2;
	short gyro_x1,gyro_x2;
	short gyro_y1,gyro_y2;
	short gyro_z1,gyro_z2;

	while(1) {
		// Tell the MPU6050 to position the internal register pointer to register
		// MPU6050_ACCEL_XOUT_H.
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS1 << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 0/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS1 << 1) | I2C_MASTER_READ, 1));

		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1,   0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1+1, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1+2, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1+3, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1+4, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1+5, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1+8, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1+9, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1+10, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1+11, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1+12, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data1+13, 1));
		//i2c_master_read(cmd, data, sizeof(data), 1);
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 0/portTICK_PERIOD_MS));
	
        ti1 = esp_timer_get_time()/1000;


        i2c_cmd_link_delete(cmd);


		// Tell the MPU6050 to position the internal register pointer to register
		// MPU6050_ACCEL_XOUT_H.
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS2 << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 0/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS2 << 1) | I2C_MASTER_READ, 1));

		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2,   0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2+1, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2+2, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2+3, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2+4, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2+5, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2+8, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2+9, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2+10, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2+11, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2+12, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data2+13, 1));
		//i2c_master_read(cmd, data, sizeof(data), 1);
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 0/portTICK_PERIOD_MS));
	
        ti2 = esp_timer_get_time()/1000;


        i2c_cmd_link_delete(cmd);

		accel_x1 = (data1[0] << 8) | data1[1];
		accel_y1 = (data1[2] << 8) | data1[3];
		accel_z1 = (data1[4] << 8) | data1[5];
		gyro_x1 = (data1[8] << 8) | data1[9];
		gyro_y1 = (data1[10] << 8) | data1[11];
		gyro_z1 = (data1[12] << 8) | data1[13];

		accel_x2 = (data2[0] << 8) | data2[1];
		accel_y2 = (data2[2] << 8) | data2[3];
		accel_z2 = (data2[4] << 8) | data2[5];
		gyro_x2 = (data2[8] << 8) | data2[9];
		gyro_y2 = (data2[10] << 8) | data2[11];
		gyro_z2 = (data2[12] << 8) | data2[13];
		
		printf("1. %llu accel_x: %d, accel_y: %d, accel_z: %d gyro_x: %d, gyro_y: %d, gyro_z: %d\n2. %llu accel_x: %d, accel_y: %d, accel_z: %d gyro_x: %d, gyro_y: %d, gyro_z: %d\n",ti1 , accel_x1, accel_y1, accel_z1, gyro_x1, gyro_y1, gyro_z1,ti2 , accel_x2, accel_y2, accel_z2, gyro_x2, gyro_y2, gyro_z2);
		
	}

	
} // task_hmc5883l

