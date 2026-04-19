#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

typedef struct {
    uint16_t raw_voltage;
    uint16_t raw_current;
    float battery_voltage_v;
    float battery_current_a;
} bsp_power_sample_t;

void bsp_adc_init(void);
uint16_t bsp_adc_read_channel(uint8_t adc_channel);
bsp_power_sample_t bsp_adc_read_power(void);

#endif
