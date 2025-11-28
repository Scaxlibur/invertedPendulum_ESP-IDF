#pragma once

#include "driver/ledc.h"
#include "driver/gpio.h"

#define CONTRAL_A_GPIO_NUM  GPIO_NUM_36
#define CONTRAL_B_GPIO_NUM  GPIO_NUM_37
#define DUTY_GPIO_NUM  GPIO_NUM_35

void motor_timer_init();
void motor_channel_init();
void motor_control_init();
void motor_set_duty(int16_t duty);

