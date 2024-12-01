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
	signals_init();
}
