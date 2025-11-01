#include "key.hpp"

key::key(const char *TAG, int32_t gpio_num, button_cb_t cb_func)
{
    btn_gpio_cfg.gpio_num = gpio_num;
    btn_gpio_cfg.active_level = 0;
    btn_gpio_cfg.disable_pull = false;
    this->TAG = TAG;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &gpio_btn);
    if(NULL == gpio_btn) {
        ESP_LOGE(TAG, "Button create failed");
    }

    //注册事件回调函数
    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, NULL, cb_func,NULL);

}

void key1_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key1", "BUTTON_SINGLE_CLICK");
}

void key2_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key2", "BUTTON_SINGLE_CLICK");
}

void key3_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key3", "BUTTON_SINGLE_CLICK");
}

void key4_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key4", "BUTTON_SINGLE_CLICK");
}