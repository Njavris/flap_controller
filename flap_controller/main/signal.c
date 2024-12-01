#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
#include "driver/ledc.h"

#include "signal.h"
#include "values.h"

struct pwm_dev {
	bool en_out;
	bool en_in;
	uint32_t gpio_out;
	ledc_channel_t out_ch;
	ledc_timer_t out_tmr;
	uint32_t freq;
	uint32_t pwm;
	uint32_t gpio_in;
	QueueHandle_t pwm_val_queue;
};


static void pwm_out_init(struct pwm_dev *dev) {
	static ledc_channel_t ch = LEDC_CHANNEL_0;

	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = 1 << dev->gpio_out;
	io_conf.pull_down_en = 1;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	ledc_timer_config_t ledc_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_10_BIT,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = dev->freq,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	ledc_channel_config_t ledc_channel = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = ch,
		.timer_sel = LEDC_TIMER_0,
		.intr_type = LEDC_INTR_DISABLE,
		.gpio_num = dev->gpio_out,
		.duty = 0,
		.hpoint = 0,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
	dev->out_ch = ch;
	ch ++;
}

static void pwm_out_set_duty(struct pwm_dev *dev, uint32_t duty) {
	duty = ((1 << 10) * duty) / 100;
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, dev->out_ch, duty));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, dev->out_ch));
}

#if 0
static bool pwm_cap_cb(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data) {
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

static void pwm_in_task(void *arg) {
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
	ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer,
			&cap_ch_conf, &cap_chan));

	mcpwm_capture_event_callbacks_t cbs = {
		.on_cap = pwm_cap_cb,
	};
	ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(
				cap_chan, &cbs, cur_task));

	ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));
	ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
	ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

	for (;;) {
		uint32_t duty;
		if (xTaskNotifyWait(0x00, ULONG_MAX, &duty,
				pdMS_TO_TICKS(1000)) == pdTRUE) {
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
#endif

static void IRAM_ATTR gpio_isr_handler(void* arg) {
	static int64_t rising_0 = 0;
	static int64_t falling = 0;

	struct pwm_dev *dev = (struct pwm_dev *)arg;
	uint32_t val = gpio_get_level(dev->gpio_in);
	int64_t counter = portGET_RUN_TIME_COUNTER_VALUE();
	if (!val && rising_0) {
		falling = counter;
	} else if (val) {
		if (!rising_0) {
			rising_0 = counter;
		} else if (rising_0 && falling) {
			int64_t pwm = (falling - rising_0) * 100;
			pwm /=(counter - rising_0);
			gpio_isr_handler_remove(dev->gpio_in);
			xQueueSendFromISR(dev->pwm_val_queue, &pwm, NULL);
			rising_0 = 0;
			falling = 0;
		}
	}
}

static void pwm_in_edge_setup(struct pwm_dev *dev) {
	static bool inited = false;
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.pin_bit_mask = 1 << dev->gpio_in;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);
	if (!inited) {
		gpio_install_isr_service(0);
		inited = true;
	}
	dev->pwm_val_queue = xQueueCreate(1, sizeof(uint32_t));
}

static uint32_t pwm_in_edge_measure(struct pwm_dev *dev, uint32_t timeout) {
	uint32_t ret = 0;
	if (!dev->en_in)
		return ret;
	gpio_isr_handler_add(dev->gpio_in, gpio_isr_handler, (void *)dev);
	gpio_set_intr_type(dev->gpio_in, GPIO_INTR_ANYEDGE);
	if (xQueueReceive(dev->pwm_val_queue, &ret, timeout)) {
		vTaskDelay(10);
		return ret;
	}
	return ret;
}

static int pwm_dev_init(struct pwm_dev *dev, uint32_t gpio_in,
		uint32_t gpio_out, bool en_out, bool en_in, uint32_t freq) {
	int ret = 0;
	memset(dev, 0, sizeof(struct pwm_dev));
	dev->gpio_in = gpio_in;
	dev->gpio_out = gpio_out;
	dev->en_out = en_out;
	dev->en_in = en_in;
	dev->freq = freq;

	if (dev->en_in)
		pwm_in_edge_setup(dev);
	if (dev->en_out)
		pwm_out_init(dev);
	return ret;
}

static void pwm_task(void *arg) {
	struct pwm_dev pwm_1;
	struct pwm_dev pwm_2;
//	struct pwm_dev pwm_test;
	uint32_t ch1_in, ch2_in;
	uint32_t ch1_out, ch2_out;
	uint32_t ch1_override, ch2_override;

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

//	pwm_dev_init(&pwm_test,
//		CONFIG_PWM_TEST_IN_GPIO,
//		CONFIG_PWM_TEST_OUT_GPIO,
//		true, true,
//		CONFIG_DEFAULT_PWM_FREQUENCY);

	ch1_in = pwm_in_edge_measure(&pwm_1, 50);
	ch2_in = pwm_in_edge_measure(&pwm_2, 50);

	get_value(e_nvs_ch1_dty, &ch1_out, sizeof(ch1_out));
	get_value(e_nvs_ch2_dty, &ch2_out, sizeof(ch2_out));

	for (;;) {
		uint32_t tmp_ch1_out = ch1_out;
		uint32_t tmp_ch2_out = ch2_out;
		uint32_t tmp_ch1_in = ch1_in;
		uint32_t tmp_ch2_in = ch2_in;
		get_value(e_nvs_ch1_src, &ch1_override, sizeof(ch1_override));
		get_value(e_nvs_ch2_src, &ch2_override, sizeof(ch2_override));

		tmp_ch1_in = pwm_in_edge_measure(&pwm_1, 50);
		if (tmp_ch1_in != ch1_in) {
			set_value(e_nvs_ch1_in, &tmp_ch1_in, sizeof(tmp_ch1_in));
			ch1_in = tmp_ch1_in;
		}
		tmp_ch2_in = pwm_in_edge_measure(&pwm_2, 50);
		if (tmp_ch2_in != ch2_in) {
			set_value(e_nvs_ch2_in, &tmp_ch2_in, sizeof(tmp_ch2_in));
			ch2_in = tmp_ch2_in;
		}

		vTaskDelay(10);

		if (ch1_override) {
			get_value(e_nvs_ch1_dty, &tmp_ch1_out, sizeof(ch1_out));
			if (tmp_ch1_out != ch1_out) {
				pwm_out_set_duty(&pwm_1, tmp_ch1_out);
				ch1_out = tmp_ch1_out;
			}
		}
		if (ch2_override) {
			get_value(e_nvs_ch2_dty, &tmp_ch2_out, sizeof(ch2_out));
			if (tmp_ch2_out != ch2_out) {
				pwm_out_set_duty(&pwm_2, tmp_ch2_out);
				ch2_out = tmp_ch2_out;
			}
		}

		vTaskDelay(100);
	}
}

void signals_init(void) {
	xTaskCreate(pwm_task, "PWM_task", 4096, NULL, 17, NULL);
}
