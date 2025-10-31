#include "vertical_position.hpp"

adc_continuous_handle_cfg_t ADC_handle_cfg;
adc_continuous_handle_t ADC_handle = NULL;

adc_continuous_config_t ADC_IO_config;

void ADC_init()
{
    ADC_handle_cfg.max_store_buf_size = 2048;
    ADC_handle_cfg.conv_frame_size = 256;
    ADC_handle_cfg.flags.flush_pool = 1;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&ADC_handle_cfg, &ADC_handle));
}

void ADC_IO_init()
{
    ADC_IO_config.pattern_num = 1;
    ADC_IO_config.adc_pattern = new adc_digi_pattern_config_t[1];
    ADC_IO_config.adc_pattern[0].atten = ADC_ATTEN_DB_11;
    ADC_IO_config.adc_pattern[0].channel = ADC_CHANNEL_0;   //对应GPIO1
    ADC_IO_config.adc_pattern[0].unit = ADC_UNIT_1;
    ADC_IO_config.adc_pattern[0].bit_width = ADC_BITWIDTH_12;
    ADC_IO_config.sample_freq_hz = 1000;
    ADC_IO_config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    ADC_IO_config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;

    ESP_ERROR_CHECK(adc_continuous_config(ADC_handle, &ADC_IO_config));
}