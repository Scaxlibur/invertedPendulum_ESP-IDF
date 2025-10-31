#pragma once

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#define CERCLE_NUM 408  // 电机编码器每转一圈的脉冲数
#define PCNT_HIGH_LIMIT (CERCLE_NUM*2)
#define PCNT_LOW_LIMIT  (CERCLE_NUM*-2)

#define EC11_GPIO_A 0
#define EC11_GPIO_B 2

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

    const char *TAG = "PCNT";

    public:
    PCNT(   int high_limit = PCNT_HIGH_LIMIT,
            int low_limit = PCNT_LOW_LIMIT,
            uint32_t max_glitch_ns = 1000,
            int chan_a_edge_gpio_num = EC11_GPIO_A,
            int chan_a_level_gpio_num = EC11_GPIO_B,
            int chan_b_edge_gpio_num = EC11_GPIO_B,
            int chan_b_level_gpio_num = EC11_GPIO_A,
            pcnt_watch_cb_t on_reach = pcnt_on_reach
        );
    ~PCNT();
    void print_count();

};


