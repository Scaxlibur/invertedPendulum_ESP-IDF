/**
 * @file LED.hpp
 * @brief LED指示灯控制头文件
 * @author Scaxlibur
 * @details https://docs.espressif.com/projects/esp-iot-solution/zh_CN/latest/display/led_indicator.html
 */

#pragma once

#include "led_indicator.h" 
#include "led_indicator_gpio.h"
#include "gpionum_setting.hpp"

void led_init();