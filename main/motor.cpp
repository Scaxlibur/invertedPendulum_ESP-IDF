#include "motor.hpp"

ledc_timer_config_t motor_timer_cfg;
ledc_channel_t motor_channel = LEDC_CHANNEL_0;
ledc_channel_config_t motor_channel_cfg;

gpio_config_t motor_gpio_cfg;

void motor_timer_init()
{
    motor_timer_cfg.speed_mode       = LEDC_LOW_SPEED_MODE; // 必须为低速模式
    motor_timer_cfg.timer_num        = LEDC_TIMER_0;
    motor_timer_cfg.duty_resolution  = LEDC_TIMER_8_BIT;
    motor_timer_cfg.freq_hz          = 24 * 1000;   // 电机驱动PWM频率为24kHz
    motor_timer_cfg.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer_cfg));
}

void motor_channel_init()
{
    motor_channel_cfg.gpio_num       = DUTY_GPIO_NUM; // 电机占空比控制引脚
    motor_channel_cfg.speed_mode     = LEDC_LOW_SPEED_MODE;
    motor_channel_cfg.channel        = motor_channel;
    motor_channel_cfg.intr_type      = LEDC_INTR_DISABLE;
    motor_channel_cfg.timer_sel      = LEDC_TIMER_0;
    motor_channel_cfg.duty           = 0; // 初始占空比为0
    motor_channel_cfg.hpoint         = 0;
    motor_channel_cfg.sleep_mode     = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
    motor_channel_cfg.flags.output_invert = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&motor_channel_cfg));
}

void motor_control_init()
{
    motor_gpio_cfg.pin_bit_mask = ((1ULL << CONTRAL_A_GPIO_NUM) | (1ULL << CONTRAL_B_GPIO_NUM));
    motor_gpio_cfg.mode = GPIO_MODE_OUTPUT;
    motor_gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    motor_gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    motor_gpio_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&motor_gpio_cfg));
}

void motor_set_duty(int8_t duty)
{
    if(duty >= 0)
    {
        gpio_set_level(CONTRAL_A_GPIO_NUM, 1);
        gpio_set_level(CONTRAL_B_GPIO_NUM, 0);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motor_channel));
    }else{
        gpio_set_level(CONTRAL_A_GPIO_NUM, 0);
        gpio_set_level(CONTRAL_B_GPIO_NUM, 1);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_channel, duty * -1));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motor_channel));
    }

}