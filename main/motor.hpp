#pragma once

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "gpionum_setting.hpp"

void motor_timer_init();
void motor_channel_init();
void motor_control_init();
void motor_set_duty(int16_t duty);

