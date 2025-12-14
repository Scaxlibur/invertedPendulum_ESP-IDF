#pragma once

/**
 * @file gpionum_setting.hpp
 * @brief GPIO引脚号设置
 * @author Scaxlibur
*/

#include "driver/gpio.h"   

/**
 * 编码器按钮
 */
constexpr gpio_num_t KEY1_GPIO_NUM = GPIO_NUM_11;   // 编码器2
constexpr gpio_num_t KEY2_GPIO_NUM = GPIO_NUM_5;
constexpr gpio_num_t KEY3_GPIO_NUM = GPIO_NUM_6;
constexpr gpio_num_t KEY4_GPIO_NUM = GPIO_NUM_14;   // 编码器3

/**
 * 编码器AB相
 */
constexpr gpio_num_t P_SET_ENCODER_A_GPIO_NUM = GPIO_NUM_10;    // 编码器1
constexpr gpio_num_t P_SET_ENCODER_B_GPIO_NUM = GPIO_NUM_9;
constexpr gpio_num_t I_SET_ENCODER_A_GPIO_NUM = GPIO_NUM_12;    // 编码器2
constexpr gpio_num_t I_SET_ENCODER_B_GPIO_NUM = GPIO_NUM_13;
constexpr gpio_num_t D_SET_ENCODER_A_GPIO_NUM = GPIO_NUM_21;    // 编码器3
constexpr gpio_num_t D_SET_ENCODER_B_GPIO_NUM = GPIO_NUM_47;

/**
 * 电机控制
 */
constexpr gpio_num_t CONTRAL_A_GPIO_NUM = GPIO_NUM_36;
constexpr gpio_num_t CONTRAL_B_GPIO_NUM = GPIO_NUM_37;
constexpr gpio_num_t DUTY_GPIO_NUM = GPIO_NUM_35;

/**
 * LED指示灯
 */
constexpr gpio_num_t ENCODER1_PRESS_LED_GPIO_NUM = GPIO_NUM_6;
constexpr gpio_num_t ENCODER2_PRESS_LED_GPIO_NUM = GPIO_NUM_5;
constexpr gpio_num_t ENCODER3_PRESS_LED_GPIO_NUM = GPIO_NUM_4;

constexpr gpio_num_t ENCODER1_ROLL_LED_GPIO_NUM = GPIO_NUM_7;
constexpr gpio_num_t ENCODER2_ROLL_LED_GPIO_NUM = GPIO_NUM_15;
constexpr gpio_num_t ENCODER3_ROLL_LED_GPIO_NUM = GPIO_NUM_16;

constexpr gpio_num_t STATUS1_LED_GPIO_NUM = GPIO_NUM_17;
constexpr gpio_num_t STATUS2_LED_GPIO_NUM = GPIO_NUM_18;
