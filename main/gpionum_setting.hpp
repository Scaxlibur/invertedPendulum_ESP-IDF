#pragma once

/**
 * @file gpionum_setting.hpp
 * @brief GPIO引脚号设置
 * @author Scaxlibur
*/

#include "driver/gpio.h"   

constexpr gpio_num_t KEY1_GPIO_NUM = GPIO_NUM_11;   // 编码器2
constexpr gpio_num_t KEY2_GPIO_NUM = GPIO_NUM_5;
constexpr gpio_num_t KEY3_GPIO_NUM = GPIO_NUM_6;
constexpr gpio_num_t KEY4_GPIO_NUM = GPIO_NUM_14;   // 编码器3

constexpr gpio_num_t P_SET_ENCODER_A_GPIO_NUM = GPIO_NUM_10;    // 编码器1
constexpr gpio_num_t P_SET_ENCODER_B_GPIO_NUM = GPIO_NUM_9;
constexpr gpio_num_t I_SET_ENCODER_A_GPIO_NUM = GPIO_NUM_12;    // 编码器2
constexpr gpio_num_t I_SET_ENCODER_B_GPIO_NUM = GPIO_NUM_13;
constexpr gpio_num_t D_SET_ENCODER_A_GPIO_NUM = GPIO_NUM_21;    // 编码器3
constexpr gpio_num_t D_SET_ENCODER_B_GPIO_NUM = GPIO_NUM_47;