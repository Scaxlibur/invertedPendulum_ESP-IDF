#include "rotary_encoder.hpp"

bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

PCNT::PCNT( int high_limit,
            int low_limit,
            uint32_t max_glitch_ns,
            int chan_a_edge_gpio_num,
            int chan_a_level_gpio_num,
            int chan_b_edge_gpio_num,
            int chan_b_level_gpio_num,
            pcnt_watch_cb_t on_reach,
            const char *TAG
)
{
    unit_config.high_limit = high_limit;
    unit_config.low_limit = low_limit;
    unit_config.intr_priority = 0;
    filter_config.max_glitch_ns = max_glitch_ns;
    chan_a_config.edge_gpio_num = chan_a_edge_gpio_num;
    chan_a_config.level_gpio_num = chan_a_level_gpio_num;
    chan_b_config.edge_gpio_num = chan_b_edge_gpio_num;
    chan_b_config.level_gpio_num = chan_b_level_gpio_num;
    cbs.on_reach = on_reach;
    this->TAG = TAG;

    rotary_encoder_init();
}

PCNT::~PCNT()
{
    // Destructor implementation (if needed)
}

void PCNT::rotary_encoder_init()
{
    ESP_LOGI(TAG, "install pcnt unit");

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");

    
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));


    
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    
    queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    #if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
        // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
        ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
        ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
        ESP_ERROR_CHECK(esp_light_sleep_start());
    #endif

    // Report counter value

}

void PCNT::print_count(void)
{

    if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(1000))) {
        ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
    } else {
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
        ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
    }

}

int PCNT::delta(void)
{
    int current_count;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &current_count));
    return current_count - last_call_count;
}

int PCNT::location(void)
{
    int current_count;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &current_count));
    return current_count;
}

void PCNT::print_data(void)
{
    int current_count;
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &current_count));
    ESP_LOGI(TAG, "编码器：%d", current_count);
}