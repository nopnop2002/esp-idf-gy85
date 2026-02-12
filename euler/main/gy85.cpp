// I2C device class (I2Cdev) demonstration Arduino sketch for GY85
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//		2023-03-10 - Fit to esp-idf v5
//		2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//		2016-04-18 - Eliminated a potential infinite loop
//		2013-05-08 - added seamless Fastwire support
//				   - added note about gyro calibration
//		2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//		2012-06-20 - improved FIFO overflow handling and simplified read process
//		2012-06-19 - completely rearranged DMP initialization code and simplification
//		2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//		2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//		2012-06-05 - add gravity-compensated initial reference frame acceleration output
//				   - add 3D math helper file to DMP6 example sketch
//				   - add Euler output and Yaw/Pitch/Roll output formats
//		2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//		2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//		2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <math.h>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "cJSON.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

#include "I2Cdev.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"

// Source: https://github.com/TKJElectronics/KalmanFilter
#include "Kalman.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define PI M_PI
#define RAD_TO_DEG (180.0/PI)
#define DEG_TO_RAD 0.0174533

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)

ADXL345 accel(ADXL345_DEFAULT_ADDRESS);
ITG3200 gyro(ITG3200_DEFAULT_ADDRESS);
HMC5883L mag(HMC5883L_DEFAULT_ADDRESS);
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;

// Accel & Gyro scale factor
float accel_sensitivity;
float gyro_sensitivity;
int gx_offset, gy_offset, gz_offset;

bool getMagData(int16_t *mx, int16_t *my, int16_t *mz) {
	// Wait until DataReady
	ESP_LOGD(TAG, "mag.getReadyStatus()=0x%x", mag.getReadyStatus());
	for (int retry=0;retry<10;retry++) {
		if (mag.getReadyStatus()) break;
		vTaskDelay(1);
	}

	if (mag.getReadyStatus()) {
		mag.getHeading(mx, my, mz);
		return true;
	} else {
		ESP_LOGE(TAG, "*****HMC5883L data not ready*****");
		ESP_LOGE(TAG, "mag.getReadyStatus()=0x%x", mag.getReadyStatus());
		//vTaskDelay(10);
		return false;
	}
	return false;
}

void _getMagData(double *magX, double *magY, double *magZ) {
	// Read raw data from mag. Units don't care.
	int16_t _mx, _my, _mz;
	if (getMagData(&_mx, &_my, &_mz)) {
		ESP_LOGD(TAG, "mag=%d %d %d", _mx, _my, _mz);
		_mx = _mx + CONFIG_MAGX;
		_my = _my + CONFIG_MAGY;
		_mz = _mz + CONFIG_MAGZ;
		*magX = _mx / 10.9; //uT
		*magY = _my / 10.9; //uT
		*magZ = _mz / 10.9; //uT
		ESP_LOGD(TAG, "mag=%f %f %f", *magX, *magY, *magZ);
	}
}
void _getAcceleration(double *_ax, double *_ay, double *_az) {
	// Read raw data from accel. Units don't care.
	int16_t ax, ay, az;
	accel.getAcceleration(&ax, &ay, &az);
	*_ax = ax / accel_sensitivity;
	*_ay = ay / accel_sensitivity;
	*_az = az / accel_sensitivity;
	//printf("acc=%f %f %f\n", *_ax, *_ay, *_az);
}

void gyroZeroCalibrate(int samples, int sampleDelayMS, int *x_offset, int *y_offset, int *z_offset) 
{
	int32_t x_offset_temp = 0;
	int32_t y_offset_temp = 0;
	int32_t z_offset_temp = 0;
	int16_t x,y,z;
	//gyro.getRotation(&x, &y, &z);
	for (int i = 0;i < samples;i++){
		delay(sampleDelayMS);
		gyro.getRotation(&x, &y, &z);
		x_offset_temp += x;
		y_offset_temp += y;
		z_offset_temp += z;
		// printf("offset_temp=%ld %ld %ld\n", x_offset_temp, y_offset_temp, z_offset_temp);
	}

	// printf("last offset_temp=%ld %ld %ld\n", x_offset_temp, y_offset_temp, z_offset_temp);
	x_offset_temp = x_offset_temp/samples;
	y_offset_temp = y_offset_temp/samples;
	z_offset_temp = z_offset_temp/samples;
	// printf("avr offset_temp=%ld %ld %ld\n", x_offset_temp, y_offset_temp, z_offset_temp);
	*x_offset = x_offset_temp;
	*y_offset = y_offset_temp;
	*z_offset = z_offset_temp;
	// printf("offset=%d %d %d\n", *x_offset, *y_offset, *z_offset);
}

void _getRotation(double *_gx, double *_gy, double *_gz) {
	// Read raw data from accel. Units don't care.
	int16_t gx, gy, gz;
	gyro.getRotation(&gx, &gy, &gz);
#if 0
	*_gx = gx / gyro_sensitivity;
	*_gy = gy / gyro_sensitivity;
	*_gz = gz / gyro_sensitivity;
#endif
	*_gx = (gx - gx_offset) / gyro_sensitivity;
	*_gy = (gy - gy_offset) / gyro_sensitivity;
	*_gz = (gz - gz_offset) / gyro_sensitivity;
	//printf("gyro=%f %f %f\n", *_gz, *_gy, *_gz);
}

void getRollPitch(double accX, double accY, double accZ, double *roll, double *pitch) {
	// atan2 outputs the value of - to	(radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	*roll = atan2(accY, accZ) * RAD_TO_DEG;
	*pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	*roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	*pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

// See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
void updateYaw(double magX, double magY, double magZ, double kalAngleX, double kalAngleY, double *yaw) {
	double _magX = magX * -1.0; // Invert axis - this it done here, as it should be done after the calibration
	double _magY = magY;
	double _magZ = magZ * -1.0;

	double rollAngle = kalAngleX * DEG_TO_RAD;
	double pitchAngle = kalAngleY * DEG_TO_RAD;

	//double Bfy = _magZ * sin(rollAngle) - _magY * cos(rollAngle);
	double Bfy = _magY * cos(rollAngle) - _magZ * sin(rollAngle);
	double Bfx = _magX * cos(pitchAngle) + _magY * sin(pitchAngle) * sin(rollAngle) + _magZ * sin(pitchAngle) * cos(rollAngle);
	*yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

	//yaw *= -1;
}

void gy85(void *pvParameters){
	// Initialize ADXL345
	accel.initialize(400000);

	// Verify the I2C connection
	if (!accel.testConnection()) {
		ESP_LOGE(TAG, "ADXL345 not found");
		vTaskDelete(NULL);
	}

	// Set Accelerometer Full Scale Range to ±2mg
	accel.setRange(0x00);
	accel_sensitivity = 3.9 * 1000.0; // g

	// Initialize ITG3200
	gyro.initialize(400000);

	// Verify the I2C connection
	if (!gyro.testConnection()) {
		ESP_LOGE(TAG, "ITG3200 not found");
		vTaskDelete(NULL);
	}
	gyroZeroCalibrate(200, 10, &gx_offset, &gy_offset, &gz_offset);
	ESP_LOGI(TAG, "g_offset=%d %d %d", gx_offset, gy_offset, gz_offset);
	gyro_sensitivity = 14.375; // Deg/Sec

	// Initialize HMC5883L
	// The number of samples averaged per measured output is 8.
	// Data Output Rate is 15Hz.
	// Normal measurement configuration.
	// -1.3Ga-->+1.3Ga 1090 counts / Gauss
	// Single-Measurement Mode.
	mag.initialize(400000);

	// Verify the I2C connection
	if (!mag.testConnection()) {
		ESP_LOGE(TAG, "HMC5883L not found");
		vTaskDelete(NULL);
	}

	// Verify identification byte A
	uint8_t ida = mag.getIDA();
	ESP_LOGI(TAG, "ida=0x%x", ida);
	if (ida != 0x48) {
		ESP_LOGE(TAG, "Identification Register A not correct [0x%x]", ida);
		vTaskDelete(NULL);
	}

	// Verify identification byte B
	uint8_t idb = mag.getIDB();
	ESP_LOGI(TAG, "idb=0x%x", idb);
	if (idb != 0x34) {
		ESP_LOGE(TAG, "Identification Register B not correct [0x%x]", idb);
		vTaskDelete(NULL);
	}

	// Verify identification byte C
	uint8_t idc = mag.getIDC();
	ESP_LOGI(TAG, "idc=0x%x", idc);
	if (idc != 0x33) {
		ESP_LOGE(TAG, "Identification Register C not correct [0x%x]", idc);
		vTaskDelete(NULL);
	}

	/* Set Kalman and gyro starting angle */
	double ax, ay, az;
	double gx, gy, gz;
	double mx, my, mz;
	double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer
	double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

	_getAcceleration(&ax, &ay, &az);
	_getRotation(&gx, &gy, &gz);
	getRollPitch(ax, ay, az, &roll, &pitch);
	kalAngleX = roll;
	kalAngleY = pitch;
	_getMagData(&mx, &my, &mz);
	updateYaw(mx, my, mz, kalAngleX, kalAngleY, &yaw);
	kalAngleZ = yaw;
	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	kalmanZ.setAngle(yaw);
	uint32_t timer = micros();

	int elasped = 0;
	bool initialized = false;
	double initial_kalAngleX = 0.0;
	double initial_kalAngleY = 0.0;

	while(1){
		_getAcceleration(&ax, &ay, &az);
		_getRotation(&gx, &gy, &gz);
		// printf("%f %f %f - %f %f %f\n", ax, ay, az, gx, gy, gz);
		getRollPitch(ax, ay, az, &roll, &pitch);	
		_getMagData(&mx, &my, &mz);

		double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
		timer = micros();

		/* Roll and pitch estimation */
		double gyroXrate = gx;
		double gyroYrate = gy;

#ifdef RESTRICT_PITCH
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			kalmanX.setAngle(roll);
			kalAngleX = roll;
		} else
			kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			kalmanY.setAngle(pitch);
			kalAngleY = pitch;
		} else
			kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleY) > 90)
			gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

		/* Yaw estimation */
		updateYaw(mx, my, mz, kalAngleX, kalAngleY, &yaw);
		double gyroZrate = gz;
		// This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
		if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
			kalmanZ.setAngle(yaw);
			kalAngleZ = yaw;
		} else 
			kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter

		/* Print Data every 10 times */
		if (elasped > 10) {
			// Set the first data
			if (!initialized) {
				initial_kalAngleX = kalAngleX;
				initial_kalAngleY = kalAngleY;
				initialized = true;
			}

#if 0
			printf("roll:%f", roll); printf(" ");
			printf("kalAngleX:%f", kalAngleX); printf(" ");
			printf("initial_kalAngleX:%f", initial_kalAngleX); printf(" ");
			printf("kalAngleX-initial_kalAngleX:%f", kalAngleX-initial_kalAngleX); printf(" ");
			printf("\n");

			printf("pitch: %f", pitch); printf(" ");
			printf("kalAngleY:%f", kalAngleY); printf(" ");
			printf("initial_kalAngleY: %f", initial_kalAngleY); printf(" ");
			printf("kalAngleY-initial_kalAngleY: %f", kalAngleY-initial_kalAngleY); printf(" ");
			printf("\n");

			printf("yaw:%f", yaw); printf(" ");
			printf("kalAngleZ:%f", kalAngleZ); printf(" ");
			printf("\n");
#endif

			// Send UDP packet
			float _roll = kalAngleX-initial_kalAngleX;
			float _pitch = kalAngleY-initial_kalAngleY;
			float _yaw = kalAngleZ;
			ESP_LOGI(TAG, "roll:%f pitch=%f yaw=%f", _roll, _pitch, _yaw);

			POSE_t pose;
			pose.roll = _roll;
			pose.pitch = _pitch;
			pose.yaw = _yaw;
			if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS ) {
				ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
			}

			// Send WEB request
			cJSON *request;
			request = cJSON_CreateObject();
			cJSON_AddStringToObject(request, "id", "data-request");
			cJSON_AddNumberToObject(request, "roll", _roll);
			cJSON_AddNumberToObject(request, "pitch", _pitch);
			cJSON_AddNumberToObject(request, "yaw", _yaw);
			char *my_json_string = cJSON_Print(request);
			ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
			size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			if (xBytesSent != strlen(my_json_string)) {
				ESP_LOGE(TAG, "xMessageBufferSend fail");
			}
			cJSON_Delete(request);
			cJSON_free(my_json_string);

			vTaskDelay(1);
			elasped = 0;
		}
	
		elasped++;
		vTaskDelay(1);
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}
