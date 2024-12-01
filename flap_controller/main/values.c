#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "string.h"

#include "values.h"

SemaphoreHandle_t access_smphr;

typedef struct {
	value_id_e id;
	char *key;
	uint32_t len;
	bool nvs;
	union {
		void *def;
		uint32_t val32;
	};
	void *val;
} value_t;


value_t vals[] = {
	[e_nvs_ch1_src] = { e_nvs_ch1_src, "ch1_src",
		sizeof(uint32_t), true, { NULL }, NULL },
	[e_nvs_ch2_src] = { e_nvs_ch2_src, "ch2_src",
		sizeof(uint32_t), true, { NULL }, NULL },
	[e_nvs_ch1_dty] = { e_nvs_ch1_dty, "ch1_dty",
		sizeof(uint32_t), true, { NULL }, NULL },
	[e_nvs_ch2_dty] = { e_nvs_ch2_dty, "ch2_dt",
		sizeof(uint32_t), true, { NULL }, NULL },
	[e_nvs_ch1_in] = { e_nvs_ch1_in, "ch1_in",
		sizeof(uint32_t), false, { NULL }, NULL },
	[e_nvs_ch2_in] = { e_nvs_ch2_in, "ch2_in",
		sizeof(uint32_t), false, { NULL }, NULL },


	[e_nvs_tst] = { e_nvs_tst, "tst",
		sizeof(uint32_t), true, { NULL }, NULL },
	[e_nvs_tst1] = { e_nvs_tst1, "tst1",
		-1, true, { "test" }, NULL },

	[e_nvs_max] = { },
};

static void nvs_save(value_id_e id) {
    	nvs_handle_t nvs_hndl;
	if (nvs_open("cfg", NVS_READWRITE, &nvs_hndl) != ESP_OK)
		return;
	if (nvs_set_blob(nvs_hndl, vals[id].key, vals[id].val,
			vals[id].len) != ESP_OK)
		return;

    	nvs_commit(nvs_hndl);
	nvs_close(nvs_hndl);
}

static void nvs_load(value_id_e id) {
	size_t sz = 0;
	esp_err_t err;
    	nvs_handle_t nvs_hndl;
	if (nvs_open("cfg", NVS_READWRITE, &nvs_hndl) != ESP_OK)
		return;

	err = nvs_get_blob(nvs_hndl, vals[id].key, NULL, &sz);
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
		return;

	if (sz) {
		if (sz != vals[id].len || !vals[id].len) {
			if (vals[id].val)
				free(vals[id].val);
			vals[id].val = malloc(sz);
			vals[id].len = sz;
		}
		if (nvs_get_blob(nvs_hndl, vals[id].key, vals[id].val,
				&sz) != ESP_OK)
			return;
	}
	nvs_close(nvs_hndl);
}

int get_value(value_id_e id, void *val, int *len) {
	for (;;) { 
		if (xSemaphoreTake(access_smphr, portMAX_DELAY)) {
			if (!vals[id].val) {
				xSemaphoreGive(access_smphr);
				return -1;
			}
			memcpy(val, vals[id].val, vals[id].len);
			if (len)
				*len = vals[id].len;
			xSemaphoreGive(access_smphr);
			return 0;
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	return 0;
}

void set_value(value_id_e id, void *val, int len) {
	for (;;) {
		if (xSemaphoreTake(access_smphr, portMAX_DELAY)) {
			if (!vals[id].val) {
				vals[id].val = malloc(len);
				vals[id].len = len;
			}
			memcpy(vals[id].val, val, len);
			vals[id].len = len;
			if (vals[id].nvs)
				nvs_save(id);
			xSemaphoreGive(access_smphr);
			return;
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void values_init(void) {
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

	for (int i = 0; i < e_nvs_max; i++) {
		if (vals[i].nvs) {
			nvs_load(vals[i].id);
		}
	}
}
