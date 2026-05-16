#include "modules/control/mixer.h"
#include "app/app_config.h"

static float clamp_norm(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }

    if (value > 1.0f) {
        return 1.0f;
    }

    return value;
}

static uint16_t norm_to_pwm(float norm)
{
    const float safe_norm = clamp_norm(norm);
    const uint16_t span = FC_MOTOR_BRINGUP_LIMIT_US - FC_MOTOR_PWM_MIN_US;
    return (uint16_t)(FC_MOTOR_PWM_MIN_US + (uint16_t)(safe_norm * (float)span));
}

void mixer_init(void)
{
}

void mixer_mix_x_quad(const control_setpoint_t *setpoint, bool armed, motor_outputs_t *outputs)
{
    if (outputs == 0) {
        return;
    }

    if ((!armed) || (setpoint == 0)) {
        for (uint8_t i = 0; i < FC_MOTOR_COUNT; i++) {
            outputs->motor_us[i] = FC_MOTOR_PWM_MIN_US;
        }
        return;
    }

    const float t = setpoint->throttle_norm;
    const float r = setpoint->roll_cmd * 0.0015f;
    const float p = setpoint->pitch_cmd * 0.0015f;
    const float y = setpoint->yaw_cmd * 0.0010f;

    /* X-quad order:
     * M1 front-left,  M2 front-right, M3 rear-right, M4 rear-left.
     * Sign convention is intentionally isolated here for prop-off checks.
     */
    outputs->motor_us[0] = norm_to_pwm(t + p + r - y);
    outputs->motor_us[1] = norm_to_pwm(t + p - r + y);
    outputs->motor_us[2] = norm_to_pwm(t - p - r - y);
    outputs->motor_us[3] = norm_to_pwm(t - p + r + y);
}
