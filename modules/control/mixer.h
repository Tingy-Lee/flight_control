#ifndef MIXER_H
#define MIXER_H

#include <stdbool.h>
#include "app/app_types.h"

void mixer_init(void);
void mixer_mix_x_quad(const control_setpoint_t *setpoint, bool armed, motor_outputs_t *outputs);

#endif
