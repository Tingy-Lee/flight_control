#include "bsp/bsp_pwm.h"
#include "debug.h"

static uint16_t clamp_pulse_us(uint16_t pulse_us)
{
    if (pulse_us < FC_MOTOR_PWM_MIN_US) {
        return FC_MOTOR_PWM_MIN_US;
    }

    if (pulse_us > FC_MOTOR_PWM_MAX_US) {
        return FC_MOTOR_PWM_MAX_US;
    }

    return pulse_us;
}

void bsp_pwm_motors_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OCInitTypeDef oc = {0};
    TIM_TimeBaseInitTypeDef tb = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA | RCC_HB2Periph_GPIOE | RCC_HB2Periph_TIM1 | RCC_HB2Periph_AFIO, ENABLE);

    /* TIM1 PWM outputs:
     * M1 PA8  TIM1_CH1
     * M2 PE11 TIM1_CH2
     * M3 PE13 TIM1_CH3
     * M4 PE14 TIM1_CH4
     *
     * PA8 is used for M1 because it is a 3.3 V TIM1_CH1 pin on the EVB;
     * the PE9/PE11/PE13/PE14 bank may be powered from the lower VIO_1.8 rail.
     */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF1);

    gpio.GPIO_Pin = GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOE, &gpio);

    const uint32_t timer_hz = SystemCoreClock;
    const uint16_t prescaler = (uint16_t)((timer_hz / 1000000U) - 1U);
    const uint16_t period = (uint16_t)((1000000U / FC_MOTOR_PWM_HZ) - 1U);//根据这个配置pwm的频率就是FC_MOTOR_PWM_HZ

    tb.TIM_Period = period;
    tb.TIM_Prescaler = prescaler;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &tb);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = FC_MOTOR_PWM_MIN_US;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM1, &oc);
    TIM_OC2Init(TIM1, &oc);
    TIM_OC3Init(TIM1, &oc);
    TIM_OC4Init(TIM1, &oc);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    bsp_pwm_motors_set_all_us(FC_MOTOR_PWM_MIN_US);
}

void bsp_pwm_motor_set_us(uint8_t motor_index, uint16_t pulse_us)
{
    const uint16_t pulse = clamp_pulse_us(pulse_us);

    switch (motor_index) {
    case 0:
        TIM_SetCompare1(TIM1, pulse);
        break;
    case 1:
        TIM_SetCompare2(TIM1, pulse);
        break;
    case 2:
        TIM_SetCompare3(TIM1, pulse);
        break;
    case 3:
        TIM_SetCompare4(TIM1, pulse);
        break;
    default:
        break;
    }
}

void bsp_pwm_motors_set_all_us(uint16_t pulse_us)
{
    for (uint8_t i = 0; i < FC_MOTOR_COUNT; i++) {
        bsp_pwm_motor_set_us(i, pulse_us);
    }
}

void bsp_pwm_motors_write(const uint16_t pulse_us[FC_MOTOR_COUNT])
{
    for (uint8_t i = 0; i < FC_MOTOR_COUNT; i++) {
        bsp_pwm_motor_set_us(i, pulse_us[i]);
    }
}

void bsp_pwm_motors_debug()
{
    static uint32_t debug_calls;
    uint16_t pulse_us = FC_MOTOR_PWM_MIN_US;

    debug_calls++;

    if (debug_calls > (FC_CONTROL_TASK_HZ * 5U)) {
        const uint32_t ramp_calls = debug_calls - (FC_CONTROL_TASK_HZ * 5U);
        const uint16_t ramp_us = (uint16_t)((ramp_calls * 10U) / FC_CONTROL_TASK_HZ);

        pulse_us = FC_MOTOR_PWM_MIN_US + ramp_us;
        if (pulse_us > FC_MOTOR_BRINGUP_LIMIT_US) {
            pulse_us = FC_MOTOR_BRINGUP_LIMIT_US;
        }
    }

    bsp_pwm_motor_set_us(0U, pulse_us);
    for (uint8_t i = 1U; i < FC_MOTOR_COUNT; i++) {
        bsp_pwm_motor_set_us(i, FC_MOTOR_PWM_MIN_US);
    }
}
