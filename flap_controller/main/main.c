#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "pwm.h"

void app_main(void) {
	struct pwm_dev pwm_1, pwm_2, pwm_test;

	pwm_dev_init(&pwm_1,
		CONFIG_PWM_CH1_IN_GPIO,
		CONFIG_PWM_CH1_OUT_GPIO,
		true, true,
		CONFIG_DEFAULT_PWM_FREQUENCY);

	pwm_dev_init(&pwm_2,
		CONFIG_PWM_CH2_IN_GPIO,
		CONFIG_PWM_CH2_OUT_GPIO,
		true, true,
		CONFIG_DEFAULT_PWM_FREQUENCY);

	pwm_dev_init(&pwm_test,
		CONFIG_PWM_TEST_IN_GPIO,
		CONFIG_PWM_TEST_OUT_GPIO,
		true, true,
		CONFIG_DEFAULT_PWM_FREQUENCY);

	int i = 0;

	while(true) {
		uint32_t r1, r2, rt;
		uint32_t v1 = i % 100, v2 = (i + 33) % 100, vt = (i  + 66) % 100;
		pwm_out_set_duty(&pwm_1, i % 100);
		pwm_out_set_duty(&pwm_2, (i + 33) % 100);
		pwm_out_set_duty(&pwm_test, (i + 66) % 100);
		vTaskDelay(10);
		r1 = pwm_in_edge_measure(&pwm_1, 50);
		vTaskDelay(10);
		r2 = pwm_in_edge_measure(&pwm_2, 50);
		vTaskDelay(10);
		rt = pwm_in_edge_measure(&pwm_test, 50);
		vTaskDelay(10);
	    	fprintf(stdout, "set: 1:%lu%% 2:%lu%% t:%lu%%  measured: 1:%lu%% 2:%lu%% t:%lu%%\n", v1, v2, vt, r1, r2, rt);
		i++;
	}
}
