#include "modules/control/flight_controller.h"
#include "app/app_config.h"
#include "modules/control/mixer.h"
#include "modules/control/pid.h"

static pid_t g_roll_rate_pid;
static pid_t g_pitch_rate_pid;
static pid_t g_yaw_rate_pid;

static float rc_axis_to_rate(uint16_t pwm_us, float max_rate_dps)
{
    const int32_t centered = (int32_t)pwm_us - 1500;
    return ((float)centered / 500.0f) * max_rate_dps;
}

static float rc_throttle_to_norm(uint16_t pwm_us)
{
    if (pwm_us <= FC_MOTOR_PWM_MIN_US) {
        return 0.0f;
    }

    if (pwm_us >= FC_MOTOR_PWM_MAX_US) {
        return 1.0f;
    }

    return ((float)(pwm_us - FC_MOTOR_PWM_MIN_US)) /
           ((float)(FC_MOTOR_PWM_MAX_US - FC_MOTOR_PWM_MIN_US));
}

void flight_controller_init(void)
{
    pid_init(&g_roll_rate_pid, 0.8f, 0.0f, 0.002f, -120.0f, 120.0f);
    pid_init(&g_pitch_rate_pid, 0.8f, 0.0f, 0.002f, -120.0f, 120.0f);
    pid_init(&g_yaw_rate_pid, 0.5f, 0.0f, 0.000f, -90.0f, 90.0f);
    mixer_init();
}

void flight_controller_update(flight_state_t *state, float dt_s)
{
    if (state == 0) {
        return;
    }

    state->setpoint.roll_rate_dps = rc_axis_to_rate(state->rc.roll_us, 180.0f);
    state->setpoint.pitch_rate_dps = rc_axis_to_rate(state->rc.pitch_us, 180.0f);
    state->setpoint.yaw_rate_dps = rc_axis_to_rate(state->rc.yaw_us, 120.0f);
    state->setpoint.throttle_norm = rc_throttle_to_norm(state->rc.throttle_us);

    if (!state->armed) {
        pid_reset(&g_roll_rate_pid);
        pid_reset(&g_pitch_rate_pid);
        pid_reset(&g_yaw_rate_pid);
        mixer_mix_x_quad(&state->setpoint, false, &state->motors);
        return;
    }

    const float roll_rate_meas = state->imu.gyro_dps.x;
    const float pitch_rate_meas = state->imu.gyro_dps.y;
    const float yaw_rate_meas = state->imu.gyro_dps.z;

    const float roll_cmd = pid_update(&g_roll_rate_pid,
                                      state->setpoint.roll_rate_dps - roll_rate_meas,
                                      dt_s);
    const float pitch_cmd = pid_update(&g_pitch_rate_pid,
                                       state->setpoint.pitch_rate_dps - pitch_rate_meas,
                                       dt_s);
    const float yaw_cmd = pid_update(&g_yaw_rate_pid,
                                     state->setpoint.yaw_rate_dps - yaw_rate_meas,
                                     dt_s);

    state->setpoint.roll_rate_dps = roll_cmd;
    state->setpoint.pitch_rate_dps = pitch_cmd;
    state->setpoint.yaw_rate_dps = yaw_cmd;

    mixer_mix_x_quad(&state->setpoint, true, &state->motors);
}
