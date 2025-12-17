#include "LED.hpp"

const blink_step_t active_blink_loop[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 100},              // step1: turn on LED 100 ms
    {LED_BLINK_HOLD, LED_STATE_OFF, 100},             // step2: turn off LED 100 ms
    {LED_BLINK_LOOP, 0, 0},                           // step3: loop from step1
};

const blink_step_t light_up[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 100},
    {LED_BLINK_LOOP, 0, 0},
};

typedef enum {
    ACTIVE_BLINK_LOOP, 
    LIGHT_UP, 
    BLINK_MAX,             
} led_indicator_blink_type_t;

blink_step_t const * led_indicator_blink_lists[] = {
    [ACTIVE_BLINK_LOOP] = active_blink_loop,
    [LIGHT_UP] = light_up,
    [BLINK_MAX] = NULL,
};

void led_init()
{
    /* status 1*/
    led_indicator_handle_t status1_handle = NULL;
    led_indicator_gpio_config_t status1_led_indicator_gpio_config = {
        .is_active_level_high = false,
        .gpio_num = STATUS1_LED_GPIO_NUM,
    };
    led_indicator_config_t config = {
        .blink_lists = led_indicator_blink_lists,
        .blink_list_num = BLINK_MAX,
    };
    esp_err_t ret = led_indicator_new_gpio_device(&config, &status1_led_indicator_gpio_config, &status1_handle);
}
