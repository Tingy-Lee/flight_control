#include "modules/comm/logger.h"

#include <stdio.h>

static int32_t milli(float value)
{
    return (int32_t)(value * 1000.0f);
}

void logger_init(void)
{
}

void logger_print_state(const flight_state_t *state)
{
    if (state == 0) {
        return;
    }

    printf("mode=%d armed=%d err=0x%08lx roll_mrad=%ld pitch_mrad=%ld alt_mm=%ld tgt_alt_mm=%ld tgt_vz_mmps=%ld thr=%ld m=[%u,%u,%u,%u]\r\n",
           (int)state->mode,
           state->armed ? 1 : 0,
           (unsigned long)state->error_flags,
           (long)milli(state->estimate.attitude.roll_rad),
           (long)milli(state->estimate.attitude.pitch_rad),
           (long)milli(state->estimate.altitude_m),
           (long)milli(state->setpoint.altitude_target_m),
           (long)milli(state->setpoint.vertical_speed_target_mps),
           (long)milli(state->setpoint.throttle_norm),
           state->motors.motor_us[0],
           state->motors.motor_us[1],
           state->motors.motor_us[2],
           state->motors.motor_us[3]);
}
