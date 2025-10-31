#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

class ADC
{
    private:
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t adc_init_config;
    adc_oneshot_chan_cfg_t adc_config;
    adc_channel_t chan;

    int adcbuffer;
    
    const char *TAG = "ADC";

    public:
    ADC(   adc_unit_t unit_id = ADC_UNIT_1, 
                adc_channel_t chan = ADC_CHANNEL_0, 
                adc_atten_t atten = ADC_ATTEN_DB_12, 
                adc_bitwidth_t bitwidth = ADC_BITWIDTH_DEFAULT
            );

    ~ADC();
    void init();
    void print_data();
};
