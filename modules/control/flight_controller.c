#include "modules/control/flight_controller.h"
#include "app/app_config.h"
#include "modules/control/mixer.h"
#include "modules/control/pid.h"
#include <math.h>

static pid_t g_roll_rate_pid;
static pid_t g_pitch_rate_pid;
static pid_t g_yaw_rate_pid;
static pid_t g_vertical_speed_pid;

static bool g_altitude_reference_valid;
static float g_altitude_reference_m;
static float g_hover_delta_reference_m;
static flight_mode_t g_last_mode = FLIGHT_MODE_LOCKED;

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
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

static float attitude_to_rate(float target_rad, float measured_rad, float max_rate_dps)
{
    const float rate_dps = (target_rad - measured_rad) * FC_ATTITUDE_KP_DPS_PER_RAD;
    return clamp_float(rate_dps, -max_rate_dps, max_rate_dps);
}

static void clear_rate_commands(control_setpoint_t *setpoint)
{
    if (setpoint == 0) {
        return;
    }

    setpoint->roll_cmd = 0.0f;
    setpoint->pitch_cmd = 0.0f;
    setpoint->yaw_cmd = 0.0f;
}

static void reset_rate_controllers(void)
{
    pid_reset(&g_roll_rate_pid);
    pid_reset(&g_pitch_rate_pid);
    pid_reset(&g_yaw_rate_pid);
}

static void reset_altitude_controller(void)
{
    pid_reset(&g_vertical_speed_pid);
    g_altitude_reference_valid = false;
}

static void motors_set_all(motor_outputs_t *outputs, uint16_t pulse_us)
{
    if (outputs == 0) {
        return;
    }

    for (uint8_t i = 0; i < FC_MOTOR_COUNT; i++) {
        outputs->motor_us[i] = pulse_us;
    }
}

static void latch_altitude_reference(const flight_state_t *state)
{
    g_altitude_reference_m = state->estimate.altitude_m;
    g_hover_delta_reference_m = state->target.hover_height_delta_m;
    g_altitude_reference_valid = true;
}

static float altitude_error_to_vertical_speed(float error_m)
{
    const float vertical_speed_mps = error_m * FC_ALTITUDE_KP_MPS_PER_M;
    return clamp_float(vertical_speed_mps,
                       -FC_ALTITUDE_MAX_DESCENT_RATE_MPS,
                       FC_ALTITUDE_MAX_CLIMB_RATE_MPS);
}

static float apply_tilt_compensation(const flight_state_t *state, float throttle_norm)
{
    if (!state->estimate.attitude_valid) {
        return throttle_norm;
    }

    const float cos_roll = cosf(state->estimate.attitude.roll_rad);
    const float cos_pitch = cosf(state->estimate.attitude.pitch_rad);
    float vertical_gain = cos_roll * cos_pitch;

    if (vertical_gain < FC_ALTITUDE_TILT_COMP_MIN_COS) {
        vertical_gain = FC_ALTITUDE_TILT_COMP_MIN_COS;
    }

    return throttle_norm / vertical_gain;
}

static float altitude_hold_throttle(flight_state_t *state, float dt_s)
{
    float altitude_target_m = state->estimate.altitude_m;
    float pilot_vertical_speed_mps = 0.0f;

    if (!state->estimate.altitude_valid) {
        reset_altitude_controller();
        state->setpoint.altitude_target_m = state->estimate.altitude_m;
        state->setpoint.vertical_speed_target_mps = 0.0f;
        return 0.0f;
    }

    switch (state->mode) {
    case FLIGHT_MODE_TAKEOFF:
        if (!state->mode_altitude_valid) {
            reset_altitude_controller();
            return 0.0f;
        }
        altitude_target_m = state->mode_target_altitude_m;
        pilot_vertical_speed_mps = FC_TAKEOFF_CLIMB_RATE_MPS;
        break;

    case FLIGHT_MODE_ALT_HOLD:
        if (state->target.valid) {
            if (!g_altitude_reference_valid) {
                latch_altitude_reference(state);
            }

            altitude_target_m =
                g_altitude_reference_m + (state->target.hover_height_delta_m - g_hover_delta_reference_m);
            pilot_vertical_speed_mps =
                clamp_float(state->target.hover_height_rate_mps,
                            -FC_ALTITUDE_MAX_DESCENT_RATE_MPS,
                            FC_ALTITUDE_MAX_CLIMB_RATE_MPS);
        } else {
            reset_altitude_controller();
        }
        break;

    case FLIGHT_MODE_LAND:
        if (!state->mode_altitude_valid) {
            reset_altitude_controller();
            return 0.0f;
        }
        altitude_target_m = state->mode_target_altitude_m;
        pilot_vertical_speed_mps = -FC_LAND_DESCENT_RATE_MPS;
        break;

    default:
        reset_altitude_controller();
        state->setpoint.altitude_target_m = state->estimate.altitude_m;
        state->setpoint.vertical_speed_target_mps = 0.0f;
        return rc_throttle_to_norm(state->rc.throttle_us);
    }

    const float altitude_error_m = altitude_target_m - state->estimate.altitude_m;
    const float hold_vertical_speed_mps = altitude_error_to_vertical_speed(altitude_error_m);
    const float vertical_speed_target_mps =
        clamp_float(pilot_vertical_speed_mps + hold_vertical_speed_mps,
                    -FC_ALTITUDE_MAX_DESCENT_RATE_MPS,
                    FC_ALTITUDE_MAX_CLIMB_RATE_MPS);
    const float vertical_speed_error_mps =
        vertical_speed_target_mps - state->estimate.vertical_speed_mps;
    const float throttle_correction_norm =
        pid_update(&g_vertical_speed_pid, vertical_speed_error_mps, dt_s);

    float throttle_norm = FC_ALTITUDE_HOVER_THROTTLE_NORM + throttle_correction_norm;
    throttle_norm = apply_tilt_compensation(state, throttle_norm);

    state->setpoint.altitude_target_m = altitude_target_m;
    state->setpoint.vertical_speed_target_mps = vertical_speed_target_mps;

    return clamp_float(throttle_norm, 0.0f, 1.0f);
}

void flight_controller_init(void)
{
    pid_init(&g_roll_rate_pid, 0.8f, 0.0f, 0.002f, -120.0f, 120.0f);
    pid_init(&g_pitch_rate_pid, 0.8f, 0.0f, 0.002f, -120.0f, 120.0f);
    pid_init(&g_yaw_rate_pid, 0.5f, 0.0f, 0.000f, -90.0f, 90.0f);
    pid_init(&g_vertical_speed_pid,
             FC_ALTITUDE_RATE_KP_NORM_PER_MPS,
             FC_ALTITUDE_RATE_KI_NORM_PER_MPS,
             FC_ALTITUDE_RATE_KD_NORM_PER_MPS,
             -FC_ALTITUDE_RATE_OUTPUT_LIMIT_NORM,
             FC_ALTITUDE_RATE_OUTPUT_LIMIT_NORM);
    reset_altitude_controller();
    mixer_init();
}

void flight_controller_update(flight_state_t *state, float dt_s)
{
    if (state == 0) {
        return;
    }

    if (state->mode != g_last_mode) {
        reset_altitude_controller();
        g_last_mode = state->mode;
    }

    if (state->target.valid && state->estimate.attitude_valid) {
        state->setpoint.roll_rate_dps =
            attitude_to_rate(state->target.roll_rad,
                                   state->estimate.attitude.roll_rad,
                                   FC_TARGET_MAX_ROLL_RATE_DPS);
        state->setpoint.pitch_rate_dps =
            attitude_to_rate(state->target.pitch_rad,
                                   state->estimate.attitude.pitch_rad,
                                   FC_TARGET_MAX_PITCH_RATE_DPS);
    } else {
        state->setpoint.roll_rate_dps = 0.0f;
        state->setpoint.pitch_rate_dps = 0.0f;
    }

    state->setpoint.yaw_rate_dps = state->target.valid ?
                                   clamp_float(state->target.yaw_rate_dps,
                                               -FC_TARGET_MAX_YAW_RATE_DPS,
                                               FC_TARGET_MAX_YAW_RATE_DPS) :
                                   0.0f;
    state->setpoint.throttle_norm = 0.0f;
    clear_rate_commands(&state->setpoint);

    if (!state->armed) {
        reset_rate_controllers();
        reset_altitude_controller();
        mixer_mix_x_quad(&state->setpoint, false, &state->motors);
        return;
    }

    if (state->mode == FLIGHT_MODE_ARMED_IDLE) {
        reset_rate_controllers();
        reset_altitude_controller();
        state->setpoint.throttle_norm = 0.0f;
        motors_set_all(&state->motors, FC_MOTOR_PWM_IDLE_US);
        return;
    }

    state->setpoint.throttle_norm = altitude_hold_throttle(state, dt_s);

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

    state->setpoint.roll_cmd = roll_cmd;
    state->setpoint.pitch_cmd = pitch_cmd;
    state->setpoint.yaw_cmd = yaw_cmd;

    mixer_mix_x_quad(&state->setpoint, true, &state->motors);
}
