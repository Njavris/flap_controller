#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

#include "settings.h"

SemaphoreHandle_t access_smphr;



void get_setting(enum setting_id, uint8_t *val, int len) {
	for (;;) {
		if (xSemaphoreTake(access_smphr, portMAX_DELAY)) {
			xSemaphoreGive(access_smphr);
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void set_setting(enum setting_id, uint8_t *val, int len) {
	for (;;) {
		if (xSemaphoreTake(access_smphr, portMAX_DELAY)) {
			xSemaphoreGive(access_smphr);
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void settings_init(void) {
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
			err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// NVS partition was truncated and needs to be erased
		// Retry nvs_flash_init
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK( err );


	access_smphr = xSemaphoreCreateMutex();
}
