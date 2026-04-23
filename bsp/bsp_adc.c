#include "bsp/bsp_adc.h"
#include "debug.h"
#include "debug_diagnostics.h"
#include <stdbool.h>

#define ADC_TIMEOUT_LOOPS  100000U
#define ADC_REF_VOLTAGE    3.3f
#define ADC_MAX_COUNTS     4095.0f

/* Placeholder divider/gain constants. Replace after measuring your power
 * module. Defaults assume direct 0-3.3 V input for safe bench verification.
 */
#define BATTERY_VOLTAGE_SCALE  1.0f
#define BATTERY_CURRENT_SCALE  1.0f

static bool adc_wait_reset_calibration(void)
{
    uint32_t timeout = ADC_TIMEOUT_LOOPS;

    while (ADC_GetResetCalibrationStatus(ADC1)) {
        if (timeout-- == 0U) {
            g_dbg_bus.adc.timeout_count++;
            return false;
        }
    }

    return true;
}

static bool adc_wait_calibration(void)
{
    uint32_t timeout = ADC_TIMEOUT_LOOPS;

    while (ADC_GetCalibrationStatus(ADC1)) {
        if (timeout-- == 0U) {
            g_dbg_bus.adc.timeout_count++;
            return false;
        }
    }

    return true;
}

void bsp_adc_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    ADC_InitTypeDef adc = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_ADC1 | RCC_HB2Periph_GPIOA, ENABLE);
    RCC_ADCCLKConfig(RCC_ADCCLKSource_HCLK);
    RCC_ADCHCLKCLKAsSourceConfig(RCC_PPRE2_DIV2, RCC_HCLK_ADCPRE_DIV8);

    /* PA0 ADC0: battery voltage, PA1 ADC1: battery current. */
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    ADC_DeInit(ADC1);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    adc.ADC_OutputBuffer = ADC_OutputBuffer_Disable;
    adc.ADC_Pga = ADC_Pga_1;
    ADC_Init(ADC1, &adc);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    (void)adc_wait_reset_calibration();
    ADC_StartCalibration(ADC1);
    (void)adc_wait_calibration();
}

uint16_t bsp_adc_read_channel(uint8_t adc_channel)
{
    uint32_t timeout = ADC_TIMEOUT_LOOPS;

    ADC_RegularChannelConfig(ADC1, adc_channel, 1, ADC_SampleTime_CyclesMode5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
        if (timeout-- == 0U) {
            g_dbg_bus.adc.timeout_count++;
            return 0U;
        }
    }

    return ADC_GetConversionValue(ADC1);
}

bsp_power_sample_t bsp_adc_read_power(void)
{
    bsp_power_sample_t sample;

    sample.raw_voltage = bsp_adc_read_channel(ADC_Channel_0);
    sample.raw_current = bsp_adc_read_channel(ADC_Channel_1);

    sample.battery_voltage_v = ((float)sample.raw_voltage * ADC_REF_VOLTAGE / ADC_MAX_COUNTS) * BATTERY_VOLTAGE_SCALE;
    sample.battery_current_a = ((float)sample.raw_current * ADC_REF_VOLTAGE / ADC_MAX_COUNTS) * BATTERY_CURRENT_SCALE;

    return sample;
}
