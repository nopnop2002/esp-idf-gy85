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
#include "driver/i2c.h"
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
#define RAD_TO_DEG (180.0/PI)
#define DEG_TO_RAD 0.0174533
#define PI M_PI

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)

ADXL345 accel(ADXL345_DEFAULT_ADDRESS);
ITG3200 gyro(ITG3200_DEFAULT_ADDRESS);
HMC5883L mag(HMC5883L_DEFAULT_ADDRESS);
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magX, magY, magZ;
float toGauss = 10.*4912./32760.0;
float toTesla = 0.15;

// Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer
double roll, pitch, yaw;

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

void updateADXL345() {
	// Read raw data from accel. Units don't care.
	int16_t ax, ay, az;
	accel.getAcceleration(&ax, &ay, &az);
	accX = ax;
	accY = ay;
	accZ = az;
}

void updateITG3200() {
	// Read raw data from accel. Units don't care.
	int16_t gx, gy, gz;
	gyro.getRotation(&gx, &gy, &gz);
	gyroX = gx;
	gyroY = gy;
	gyroZ = gz;
}

void updateHMC5883L() {
	// Read raw data from mag. Units don't care.
	if (mag.getReadyStatus()) {
		int16_t mx, my, mz;
		mag.getHeading(&mx, &my, &mz);
		ESP_LOGD(TAG, "mag=%d %d %d", mx, my, mz);
		mx = mx + CONFIG_MAGX;
		my = my + CONFIG_MAGY;
		mz = mz + CONFIG_MAGZ;
		magX = mx;
		magY = my;
		magZ = mz;
		ESP_LOGD(TAG, "mag=%f %f %f", magX, magY, magZ);
	}
}

void updatePitchRoll() {
	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	roll = atan2(accY, accZ) * RAD_TO_DEG;
	pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
	magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
	magZ *= -1;

#if 0
	magX *= magGain[0];
	magY *= magGain[1];
	magZ *= magGain[2];

	magX -= magOffset[0];
	magY -= magOffset[1];
	magZ -= magOffset[2];
#endif

	double rollAngle = kalAngleX * DEG_TO_RAD;
	double pitchAngle = kalAngleY * DEG_TO_RAD;

	//double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
	double Bfy = magY * cos(rollAngle) - magZ * sin(rollAngle);
	double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
	yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

	//yaw *= -1;
}

void gy85(void *pvParameters){
	// Initialize ADXL345
	accel.initialize();

	// Verify the I2C connection
	if (!accel.testConnection()) {
		ESP_LOGE(TAG, "ADXL345 not found");
		vTaskDelete(NULL);
	}

	// Initialize ITG3200
	gyro.initialize();

	// Verify the I2C connection
	if (!gyro.testConnection()) {
		ESP_LOGE(TAG, "ITG3200 not found");
		vTaskDelete(NULL);
	}

	// Initialize HMC5883L
	// The number of samples averaged per measured output is 8.
	// Data Output Rate is 15Hz.
	// Normal measurement configuration.
	// -1.3Ga-->+1.3Ga 1090 counts / Gauss
	// Single-Measurement Mode.
	mag.initialize();

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
	updateADXL345();
	updateITG3200();
	updateHMC5883L();
	updatePitchRoll();
	updateYaw();

	kalmanX.setAngle(roll); // First set roll starting angle
	gyroXangle = roll;
	compAngleX = roll;

	kalmanY.setAngle(pitch); // Then pitch
	gyroYangle = pitch;
	compAngleY = pitch;

	kalmanZ.setAngle(yaw); // And finally yaw
	gyroZangle = yaw;
	compAngleZ = yaw;

	int elasped = 0;
	uint32_t timer = micros();

	bool initialized = false;
	double initial_roll = 0.0;
	double initial_pitch = 0.0;
	double initial_yaw = 0.0;

	while(1){
		/* Update all the IMU values */
		updateADXL345();
		updateITG3200();
		updateHMC5883L();

		double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
		timer = micros();

		/* Roll and pitch estimation */
		updatePitchRoll();
		double gyroXrate = gyroX / 131.0; // Convert to deg/s
		double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			kalmanX.setAngle(roll);
			compAngleX = roll;
			kalAngleX = roll;
			gyroXangle = roll;
		} else
			kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			kalmanY.setAngle(pitch);
			compAngleY = pitch;
			kalAngleY = pitch;
			gyroYangle = pitch;
		} else
			kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleY) > 90)
			gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

		/* Yaw estimation */
		updateYaw();
		double gyroZrate = gyroZ / 131.0; // Convert to deg/s
		// This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
		if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
			kalmanZ.setAngle(yaw);
			compAngleZ = yaw;
			kalAngleZ = yaw;
			gyroZangle = yaw;
		} else 
			kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter


		/* Estimate angles using gyro only */
		gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
		gyroYangle += gyroYrate * dt;
		gyroZangle += gyroZrate * dt;
		//gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate from the Kalman filter
		//gyroYangle += kalmanY.getRate() * dt;
		//gyroZangle += kalmanZ.getRate() * dt;

		/* Estimate angles using complimentary filter */
		compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
		compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
		compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

		// Reset the gyro angles when they has drifted too much
		if (gyroXangle < -180 || gyroXangle > 180) gyroXangle = kalAngleX;
		if (gyroYangle < -180 || gyroYangle > 180) gyroYangle = kalAngleY;
		if (gyroZangle < -180 || gyroZangle > 180) gyroZangle = kalAngleZ;

		
		/* Print Data every 10 times */
		if (elasped > 10) {
			// Set the first data
			if (!initialized) {
				initial_roll = roll;
				initial_pitch = pitch;
				initial_yaw = yaw;
				initialized = true;
			}

#if 0
			printf("roll:%f", roll); printf(" ");
			printf("initial_roll:%f", initial_roll); printf(" ");
			printf("roll-initial_roll:%f", roll-initial_roll); printf(" ");
			printf("gyroXangle:%f", gyroXangle); printf(" ");
			printf("compAngleX:%f", compAngleX); printf(" ");
			printf("kalAngleX:%f", kalAngleX); printf(" ");
			printf("\n");

			printf("pitch: %f", pitch); printf(" ");
			printf("initial_pitch: %f", initial_pitch); printf(" ");
			printf("pitch-initial_pitch: %f", pitch-initial_pitch); printf(" ");
			printf("gyroYangle:%f", gyroYangle); printf(" ");
			printf("compAngleY:%f", compAngleY); printf(" ");
			printf("kalAngleY:%f", kalAngleY); printf("\t");
			printf("\n");

			printf("yaw:%f", yaw); printf("\t");
			printf("initial_yaw: %f", initial_yaw); printf(" ");
			printf("gyroZangle:%f", gyroZangle); printf("\t");
			printf("compAngleZ:%f", compAngleZ); printf("\t");
			printf("kalAngleZ:%f", kalAngleZ); printf("\t");
			printf("\n");
#endif

			// Send UDP packet
			float _roll = roll-initial_roll;
			float _pitch = pitch-initial_pitch;
			float _yaw = yaw-initial_yaw;
			if (_yaw < -180.0) _yaw = _yaw + 360.0;
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
