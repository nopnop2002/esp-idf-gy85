#include <cstring>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/message_buffer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "cJSON.h"

extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "MAG";

// I2Cdev and HMC5883L must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "HMC5883L.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

#if 0
// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)
#endif

HMC5883L mag(HMC5883L_DEFAULT_ADDRESS);

void hmc5883l(void *pvParameters){
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
  
	while(1){
		// Read raw data from mag. Units don't care.
		if (mag.getReadyStatus()) {
			int16_t mx, my, mz;
			mag.getHeading(&mx, &my, &mz);
			ESP_LOGI(TAG, "mag=%d %d %d", mx, my, mz);
			mx = mx + CONFIG_MAGX;
			my = my + CONFIG_MAGY;
			mz = mz + CONFIG_MAGZ;

			// Send WEB request
			cJSON *request;
			request = cJSON_CreateObject();
			cJSON_AddStringToObject(request, "id", "data-request");
			cJSON_AddNumberToObject(request, "roll", mx);
			cJSON_AddNumberToObject(request, "pitch", my);
			cJSON_AddNumberToObject(request, "yaw", mz);
			char *my_json_string = cJSON_Print(request);
			ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
			size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			if (xBytesSent != strlen(my_json_string)) {
				ESP_LOGE(TAG, "xMessageBufferSend fail");
			}
			cJSON_Delete(request);
			cJSON_free(my_json_string);
		}

		vTaskDelay(10);
	}

	// Never reach here
	vTaskDelete(NULL);
}
