#pragma once

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

constexpr uint16_t CERCLE_NUM = 408;  // 电机编码器每转一圈的脉冲数
constexpr int PCNT_HIGH_LIMIT = CERCLE_NUM*10;
constexpr int PCNT_LOW_LIMIT = CERCLE_NUM*-10;

constexpr gpio_num_t MOTOR_ENCODER_GPIO_A = GPIO_NUM_38;
constexpr gpio_num_t MOTOR_ENCODER_GPIO_B = GPIO_NUM_2;


bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);

class PCNT
{
    private:
    void rotary_encoder_init();

    pcnt_unit_config_t unit_config;
    pcnt_unit_handle_t pcnt_unit = NULL;
    pcnt_glitch_filter_config_t filter_config;
    pcnt_chan_config_t chan_a_config;
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_chan_config_t chan_b_config;
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    pcnt_event_callbacks_t cbs;
    int watch_points[5] = {PCNT_LOW_LIMIT, CERCLE_NUM*-1, 0, CERCLE_NUM, PCNT_HIGH_LIMIT};
    QueueHandle_t queue;
    int pulse_count = 0;
    int event_count = 0;
    int last_call_count = 0;

    const char *TAG = "PCNT";

    public:

    PCNT(   int high_limit,
            int low_limit,
            uint32_t max_glitch_ns,
            int chan_a_edge_gpio_num,
            int chan_a_level_gpio_num,
            int chan_b_edge_gpio_num,
            int chan_b_level_gpio_num,
            pcnt_watch_cb_t on_reach
        );
    ~PCNT();
    void print_count();
    int location(); //获取当前位置
    int delta();   //计算上次调用以来的增量值
    void print_data();

};


