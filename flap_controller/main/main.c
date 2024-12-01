#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "pwm.h"
#include "settings.h"

void app_main(void) {
	settings_init();
	pwm_init();
}
