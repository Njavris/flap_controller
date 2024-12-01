#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "driver/uart.h"

#include "signal.h"
#include "values.h"

void app_main(void) {
	values_init();
//	signals_init();
	uint32_t value = 0;
	get_value(e_nvs_tst, &value, NULL);
	printf("value = %lu\n", value);
	value ++;
	printf("value = %lu\n", value);
	set_value(e_nvs_tst, &value, sizeof(value));
	printf("value = %lu\n", value);
}
