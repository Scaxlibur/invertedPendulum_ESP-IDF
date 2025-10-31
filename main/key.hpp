#pragma once

#include "iot_button.h"
#include "button_gpio.h"
#include "esp_log.h"


class key
{
    private:
    const button_config_t btn_cfg = {0};
    button_gpio_config_t btn_gpio_cfg;
    button_handle_t gpio_btn = NULL;
    const char *TAG;

    public:
    key(const char *TAG, int32_t gpio_num, button_cb_t cb_func);
    ~key();
};

void key1_press_cb(void *arg,void *usr_data);
