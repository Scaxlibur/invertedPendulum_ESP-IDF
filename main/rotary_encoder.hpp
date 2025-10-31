#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#define EXAMPLE_PCNT_HIGH_LIMIT 1023
#define EXAMPLE_PCNT_LOW_LIMIT  -1023

#define EXAMPLE_EC11_GPIO_A 0
#define EXAMPLE_EC11_GPIO_B 2

bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);

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
    int watch_points[5] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
    QueueHandle_t queue;
    int pulse_count = 0;
    int event_count = 0;

    const char *TAG = "PCNT";

    public:
    PCNT(   int high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
            int low_limit = EXAMPLE_PCNT_LOW_LIMIT,
            uint32_t max_glitch_ns = 1000,
            int chan_a_edge_gpio_num = EXAMPLE_EC11_GPIO_A,
            int chan_a_level_gpio_num = EXAMPLE_EC11_GPIO_B,
            int chan_b_edge_gpio_num = EXAMPLE_EC11_GPIO_B,
            int chan_b_level_gpio_num = EXAMPLE_EC11_GPIO_A,
            pcnt_watch_cb_t on_reach = example_pcnt_on_reach
        );
    ~PCNT();
    void print_count();

};


