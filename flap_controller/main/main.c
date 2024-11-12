#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "driver/mcpwm_cap.h"


#include "driver/ledc.h"

static void pwm_init(ledc_timer_t timer, ledc_channel_t channel,
			uint32_t freq, uint32_t gpio) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1 << gpio;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .timer_num        = timer,
        .freq_hz          = freq,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = channel,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = gpio,
        .duty           = 0,
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void pwm_set_duty(ledc_channel_t channel, uint32_t duty) {
    duty = ((1 << 10) * duty) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}


static bool pwm_cap_cb(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t rise = 0;
    static uint32_t fall = 0;
    static uint32_t rise_delta = 0;

    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t wakeup = pdFALSE;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
	uint32_t prev_rise = rise;
	rise = edata->cap_value;
	rise_delta = rise - fall;
	uint32_t duty = (rise_delta * 100) / (rise - prev_rise);
        xTaskNotifyFromISR(task_to_notify, 100 - duty, eSetValueWithOverwrite, &wakeup);
    } else {
	fall = edata->cap_value;
    }
    return wakeup == pdTRUE;
}

static void pwm_in_task(void *arg)
{
    static int group_id = 0;
    uint32_t gpio = *(uint32_t *)arg;
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();

    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = group_id,
    };
    group_id++;
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
	.intr_priority = 1,
        .gpio_num = gpio,
        .prescale = 1,
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        .flags.pull_up = false,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = pwm_cap_cb,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    for (;;) {
    	uint32_t duty;
	if (xTaskNotifyWait(0x00, ULONG_MAX, &duty, pdMS_TO_TICKS(1000)) == pdTRUE) {
		fprintf(stdout, "Duty: %ld%%\n", duty);
        }
	vTaskDelay(100);
    }
}

void pwm_in_mcpwm_setup(uint32_t gpio) {
    char name[16];
    sprintf(name, "pwm_in_gpio%ld", gpio);
    xTaskCreate(pwm_in_task, name, 2048, &gpio, 10, NULL);
}

static QueueHandle_t pwm_val_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    static int64_t rising_0 = 0;
    static int64_t falling = 0;

    uint32_t gpio = (uint32_t) arg;
    uint32_t val = gpio_get_level(gpio);
    int64_t counter = portGET_RUN_TIME_COUNTER_VALUE();

    if (!val && rising_0) {
	falling = counter;
    } else if (val) {
	if (!rising_0) {
	    rising_0 = counter;
	} else if (rising_0 && falling) {
	    int64_t pwm = (falling - rising_0) * 100;
	    pwm /=(counter - rising_0);
    	    gpio_isr_handler_remove(gpio);
    	    xQueueSendFromISR(pwm_val_queue, &pwm, NULL);
	    rising_0 = 0;
	    falling = 0;
	}
    }
}

void pwm_in_edge_setup(uint32_t gpio) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = 1 << gpio;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_set_intr_type(gpio, GPIO_INTR_ANYEDGE);
    pwm_val_queue = xQueueCreate(1, sizeof(uint32_t));
    gpio_install_isr_service(0);
}

uint32_t pwm_in_edge_measure(uint32_t gpio, uint32_t timeout) {
    uint32_t pwm = 0;
    gpio_isr_handler_add(gpio, gpio_isr_handler, (void *)gpio);
    if (xQueueReceive(pwm_val_queue, &pwm, timeout))
	return pwm;
    return pwm;
}

void app_main(void) {
	pwm_init(LEDC_TIMER_0, LEDC_CHANNEL_0, CONFIG_PWM_FREQUENCY, CONFIG_PWM_OUT_GPIO);
	pwm_init(LEDC_TIMER_1, LEDC_CHANNEL_1, CONFIG_PWM_FREQUENCY, CONFIG_PWM_IN_TEST_GPIO);
	pwm_set_duty(LEDC_CHANNEL_0, 10);
	pwm_set_duty(LEDC_CHANNEL_1, 90);

	fprintf(stdout, "PWM_IN_GPIO:%d\n"
			"PWM_IN_TEST_GPIO:%d\n"
			"PWM_OUT_GPIO:%d\n"
			"PWM_OUT_TEST_GPIO:%d\n"
			"PWM_FREQUENCY:%d\n",
			CONFIG_PWM_IN_GPIO,
			CONFIG_PWM_IN_TEST_GPIO,
			CONFIG_PWM_OUT_GPIO,
			CONFIG_PWM_OUT_TEST_GPIO,
			CONFIG_PWM_FREQUENCY);
	int i = 0;
	//pwm_in_mcpwm_setup(CONFIG_PWM_IN_GPIO);

	pwm_in_edge_setup(CONFIG_PWM_IN_GPIO);
	while(true) {
		uint32_t pwm;
		uint32_t val = i % 100;
		pwm_set_duty(LEDC_CHANNEL_1, val);
		vTaskDelay(10);
		pwm = pwm_in_edge_measure(CONFIG_PWM_IN_GPIO, 50);
	    	fprintf(stdout, "set: %lu%% measured: %lu%%\n", val, pwm);
		i++;
	}
}
