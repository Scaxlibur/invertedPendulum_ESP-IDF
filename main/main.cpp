#include "rotary_encoder.hpp"
#include "vertical_position.hpp"

PCNT rotary_encoder(EXAMPLE_PCNT_HIGH_LIMIT, EXAMPLE_PCNT_LOW_LIMIT, 1000, EXAMPLE_EC11_GPIO_A, EXAMPLE_EC11_GPIO_B, EXAMPLE_EC11_GPIO_B, EXAMPLE_EC11_GPIO_A, example_pcnt_on_reach);


void rotary_encoder_task(void *arg)
{
    while (true)
    {
        rotary_encoder.print_count();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void ADC_task(void *arg)
{
    ESP_ERROR_CHECK(adc_continuous_start(ADC_handle));
    while (true)
    {
        // ADC reading and processing logic
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    xTaskCreatePinnedToCore(rotary_encoder_task, "rotary_encoder_task", 4096, NULL, 5, NULL, 1);   
}