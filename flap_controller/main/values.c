#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "string.h"

#include "values.h"

SemaphoreHandle_t access_smphr;

typedef struct {
	value_id_e id;
	uint32_t len;
	bool nvs;
	uint32_t def;
	union {
		uint32_t val32;
		uint8_t *pval;
	};
} value_t;


value_t vals[] = {
	[e_nvs_ch1_src] = { e_nvs_ch1_src,
		sizeof(uint32_t), true, 0, { .val32 = 0 } },
	[e_nvs_ch2_src] = { e_nvs_ch2_src,
		sizeof(uint32_t), true, 0, { .val32 = 0 } },
	[e_nvs_ch1_dty] = { e_nvs_ch1_dty,
		sizeof(uint32_t), true, 0, { .val32 = 0 } },
	[e_nvs_ch2_dty] = { e_nvs_ch2_dty,
		sizeof(uint32_t), true, 0, { .val32 = 0 } },
	[e_nvs_ch1_in] = { e_nvs_ch1_in,
		sizeof(uint32_t), false, 0, { .val32 = 0 } },
	[e_nvs_ch2_in] = { e_nvs_ch2_in,
		sizeof(uint32_t), false, 0, { .val32 = 0 } },

	[e_nvs_tst] = { e_nvs_tst,
		sizeof(uint32_t), false, 0, { .val32 = 0 } },
};

void get_value(value_id_e id, void *val, int len) {
	for (;;) { 
		if (xSemaphoreTake(access_smphr, portMAX_DELAY)) {
			if (len == sizeof(uint32_t))
				*(uint32_t *)val = vals[id].val32;
			else 
				memcpy(vals[id].pval, val, len);
			xSemaphoreGive(access_smphr);
			return;
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void set_value(value_id_e id, void *val, int len) {
	for (;;) {
		if (xSemaphoreTake(access_smphr, portMAX_DELAY)) {
			if (len == sizeof(uint32_t))
				vals[id].val32 = *(uint32_t *)val;
			else 
				memcpy(val, vals[id].pval, len);
			xSemaphoreGive(access_smphr);
			return;
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void values_init(void) {
//	esp_err_t err = nvs_flash_init();
//	if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
//			err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//		// NVS partition was truncated and needs to be erased
//		// Retry nvs_flash_init
//		ESP_ERROR_CHECK(nvs_flash_erase());
//		err = nvs_flash_init();
//	}
//	ESP_ERROR_CHECK( err );
	access_smphr = xSemaphoreCreateMutex();
}
