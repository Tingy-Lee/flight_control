#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>
#include "app/app_config.h"

void bsp_pwm_motors_init(void);
void bsp_pwm_motor_set_us(uint8_t motor_index, uint16_t pulse_us);
void bsp_pwm_motors_set_all_us(uint16_t pulse_us);
void bsp_pwm_motors_write(const uint16_t pulse_us[FC_MOTOR_COUNT]);

#endif
