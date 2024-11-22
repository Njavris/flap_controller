#ifndef __PWM_H__
#define __PWM_H__

#include "driver/ledc.h"

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

int pwm_dev_init(struct pwm_dev *dev, uint32_t gpio_in,
		uint32_t gpio_out, bool en_out, bool en_in, uint32_t freq);
uint32_t pwm_in_edge_measure(struct pwm_dev *dev, uint32_t timeout);
void pwm_out_set_duty(struct pwm_dev *dev, uint32_t duty);

#endif // __PWM_H__
