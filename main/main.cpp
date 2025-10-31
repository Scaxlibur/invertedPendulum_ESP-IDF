#include "rotary_encoder.hpp"
#include "adc.hpp"
#include "motor.hpp"
#include "key.hpp"

void rotary_encoder_task(void *arg)
{
    PCNT rotary_encoder(EXAMPLE_PCNT_HIGH_LIMIT, EXAMPLE_PCNT_LOW_LIMIT, 1000, EXAMPLE_EC11_GPIO_A, EXAMPLE_EC11_GPIO_B, EXAMPLE_EC11_GPIO_B, EXAMPLE_EC11_GPIO_A, example_pcnt_on_reach);
    while (true)
    {
        rotary_encoder.print_count();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void ADC_task(void *arg)
{
    ADC vertical_position;
    while (true)
    {
        vertical_position.print_data();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void motor_task(void *arg)
{
    motor_timer_init();
    motor_channel_init();
    motor_control_init();
    motor_set_duty(128);
    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    }
}

void key_task(void *arg)
{
    key start_key("start_key", GPIO_NUM_14, key1_press_cb);
    while(true)
    {
        vTaskDelay(portMAX_DELAY);
    }
}

extern "C" void app_main(void)
{
    xTaskCreate(rotary_encoder_task, "rotary_encoder_task", 4096, NULL, 5, NULL);
    xTaskCreate(ADC_task, "ADC_task", 4096, NULL, 5, NULL); 
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
    xTaskCreate(key_task, "key_task", 4096, NULL, 5, NULL);
}