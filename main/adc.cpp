#include "adc.hpp"

const char *TAG = "ADC";

ADC::ADC(adc_unit_t unit_id, adc_channel_t chan, adc_atten_t atten, adc_bitwidth_t bitwidth)
{
    adc_init_config.unit_id = unit_id;
    adc_init_config.ulp_mode = ADC_ULP_MODE_DISABLE;
    adc_config.atten = atten;
    adc_config.bitwidth = bitwidth;
    this -> chan = chan;
    init();
}

void ADC::init()
{

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc_handle));
    adc_oneshot_config_channel(adc_handle, chan, &adc_config);
}

void ADC::print_data()
{
    adc_oneshot_read(adc_handle, chan, &adcbuffer);
    ESP_LOGI(TAG, "ADCdata is : %d", adcbuffer);
}   