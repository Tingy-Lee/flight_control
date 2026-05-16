#include "tasks/Inc/tasks.h"

#include "app/app.h"
#include "app/app_config.h"
#include "debug_diagnostics.h"

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

static uint16_t rc_channel_us(const rc_input_t *rc, uint8_t channel)
{
    if ((rc == 0) || (channel >= FC_RC_INPUT_CHANNEL_COUNT)) {
        return FC_RC_PWM_CENTER_US;
    }

    return rc->channels_us[channel];
}

static float rc_channel_to_axis(uint16_t pwm_us)
{
    const int32_t centered = (int32_t)pwm_us - (int32_t)FC_RC_PWM_CENTER_US;
    const int32_t deadzone = (int32_t)FC_RC_PWM_DEADZONE_US;

    if ((centered >= -deadzone) && (centered <= deadzone)) {
        return 0.0f;
    }

    return clamp_float((float)centered / (float)FC_RC_PWM_HALF_RANGE_US, -1.0f, 1.0f);
}

static float rc_hover_channel_to_axis(uint16_t pwm_us)
{
    const int32_t centered = (int32_t)pwm_us - (int32_t)FC_RC_PWM_CENTER_US;
    const int32_t deadband = (int32_t)FC_RC_HOVER_DEADBAND_US;
    const int32_t span = (int32_t)FC_RC_PWM_HALF_RANGE_US - deadband;

    if ((centered >= -deadband) && (centered <= deadband)) {
        return 0.0f;
    }

    if (span <= 0) {
        return (centered > 0) ? 1.0f : -1.0f;
    }

    if (centered > deadband) {
        return clamp_float((float)(centered - deadband) / (float)span, 0.0f, 1.0f);
    }

    return clamp_float((float)(centered + deadband) / (float)span, -1.0f, 0.0f);
}

static void decision_reset_target(target_state_t *target, uint32_t timestamp_ms)
{
    if (target == 0) {
        return;
    }

    target->timestamp_ms = timestamp_ms;
    target->roll_rad = 0.0f;
    target->pitch_rad = 0.0f;
    target->yaw_rate_dps = 0.0f;
    target->hover_height_delta_m = 0.0f;
    target->hover_height_rate_mps = 0.0f;
    target->valid = false;
}

static void decision_update_target(flight_state_t *state, float dt_s)
{
    const float roll_axis = rc_channel_to_axis(rc_channel_us(&state->rc, FC_RC_CHANNEL_ROLL));
    const float pitch_axis = rc_channel_to_axis(rc_channel_us(&state->rc, FC_RC_CHANNEL_PITCH));
    const float yaw_axis = rc_channel_to_axis(rc_channel_us(&state->rc, FC_RC_CHANNEL_YAW));
    const float hover_axis = rc_hover_channel_to_axis(rc_channel_us(&state->rc, FC_RC_CHANNEL_HOVER_HEIGHT));

    state->target.timestamp_ms = state->rc.timestamp_ms;
    state->target.roll_rad = roll_axis * FC_TARGET_MAX_ROLL_RAD;
    state->target.pitch_rad = pitch_axis * FC_TARGET_MAX_PITCH_RAD;
    state->target.yaw_rate_dps = yaw_axis * FC_TARGET_MAX_YAW_RATE_DPS;
    state->target.hover_height_rate_mps = hover_axis * FC_TARGET_MAX_HOVER_RATE_MPS;
    state->target.hover_height_delta_m =
        clamp_float(state->target.hover_height_delta_m + (state->target.hover_height_rate_mps * dt_s),
                    FC_TARGET_HOVER_DELTA_MIN_M,
                    FC_TARGET_HOVER_DELTA_MAX_M);
    state->target.valid = true;
}

void DecisionTask(void *argument)
{
    (void)argument;
    g_dbg_boot.startup_phase = 105U;
    g_dbg_boot.task_entry_mask |= (1UL << TASK_INDEX_DECISION);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = task_period_ticks(FC_DECISION_TASK_HZ);
    const float dt_s = 1.0f / (float)FC_DECISION_TASK_HZ;
    flight_state_t *state = app_state();

    while (1) {
        const TickType_t loop_start = xTaskGetTickCount();

        if (state->rc.healthy && !state->rc.failsafe) {
            decision_update_target(state, dt_s);
        } else {
            decision_reset_target(&state->target, state->rc.timestamp_ms);
        }

        task_record_heartbeat(TASK_INDEX_DECISION, loop_start);
        vTaskDelayUntil(&last_wake, period);
    }
}
