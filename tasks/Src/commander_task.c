#include "tasks/Inc/tasks.h"

#include "app/app.h"
#include "app/app_config.h"
#include "bsp/bsp_board.h"
#include "debug_diagnostics.h"
#include "modules/navigation/mission.h"
#include "modules/safety/safety.h"

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool attitude_is_safe_to_arm(const flight_state_t *state)
{
    if ((state == 0) || (!state->estimate.attitude_valid)) {
        return false;
    }

    return (abs_float(state->estimate.attitude.roll_rad) <= FC_ARM_ATTITUDE_MAX_RAD) &&
           (abs_float(state->estimate.attitude.pitch_rad) <= FC_ARM_ATTITUDE_MAX_RAD);
}

static bool controls_are_safe_to_arm(const flight_state_t *state)
{
    if ((state == 0) || (!state->rc.healthy) || state->rc.failsafe) {
        return false;
    }

    return state->rc.arm_switch && (state->rc.throttle_us <= FC_ARM_THROTTLE_MAX_US);
}

static bool takeoff_requested(const flight_state_t *state)
{
    return (state != 0) &&
           state->rc.healthy &&
           state->rc.arm_switch &&
           (state->rc.throttle_us >= FC_TAKEOFF_TRIGGER_US);
}

static bool only_rc_failsafe(uint32_t errors)
{
    return (errors != 0U) && ((errors & ~FC_ERR_RC_FAILSAFE) == 0U);
}

static void enter_mode(flight_state_t *state, flight_mode_t next_mode)
{
    const uint32_t now_ms = bsp_board_millis();
    const bool altitude_valid = state->estimate.altitude_valid;
    const float current_altitude_m = state->estimate.altitude_m;

    if ((state->mode == next_mode) && (state->mode_entry_ms != 0U)) {
        return;
    }

    state->mode = next_mode;
    state->mode_entry_ms = now_ms;
    state->mode_start_altitude_m = current_altitude_m;
    state->mode_target_altitude_m = current_altitude_m;
    state->mode_altitude_valid = altitude_valid;

    switch (next_mode) {
    case FLIGHT_MODE_LOCKED:
    case FLIGHT_MODE_STANDBY:
    case FLIGHT_MODE_FAILSAFE:
        state->armed = false;
        state->home_altitude_valid = false;
        state->mode_altitude_valid = false;
        break;

    case FLIGHT_MODE_ARMED_IDLE:
        state->armed = true;
        if (altitude_valid) {
            state->home_altitude_m = current_altitude_m;
            state->home_altitude_valid = true;
        }
        break;

    case FLIGHT_MODE_TAKEOFF:
        state->armed = true;
        if (altitude_valid && !state->home_altitude_valid) {
            state->home_altitude_m = current_altitude_m;
            state->home_altitude_valid = true;
        }
        if (state->home_altitude_valid) {
            state->mode_start_altitude_m = current_altitude_m;
            state->mode_target_altitude_m = state->home_altitude_m + FC_TAKEOFF_ALTITUDE_M;
            state->mode_altitude_valid = true;
        }
        break;

    case FLIGHT_MODE_ALT_HOLD:
        state->armed = true;
        if (altitude_valid) {
            state->mode_target_altitude_m = current_altitude_m;
            state->mode_altitude_valid = true;
        }
        break;

    case FLIGHT_MODE_LAND:
        state->armed = true;
        if (state->home_altitude_valid) {
            state->mode_target_altitude_m = state->home_altitude_m;
            state->mode_altitude_valid = true;
        } else {
            state->mode_altitude_valid = altitude_valid;
        }
        break;

    case FLIGHT_MODE_MANUAL_STAB:
    case FLIGHT_MODE_MISSION:
    default:
        state->armed = true;
        break;
    }
}

static bool takeoff_complete(const flight_state_t *state)
{
    if ((state == 0) || (!state->estimate.altitude_valid) || (!state->mode_altitude_valid)) {
        return false;
    }

    return state->estimate.altitude_m >=
           (state->mode_target_altitude_m - FC_TAKEOFF_COMPLETE_MARGIN_M);
}

static bool land_complete(const flight_state_t *state)
{
    const uint32_t now_ms = bsp_board_millis();
    float ground_altitude_m = 0.0f;

    if ((state == 0) || (!state->estimate.altitude_valid)) {
        return false;
    }

    ground_altitude_m = state->home_altitude_valid ?
                        state->home_altitude_m :
                        state->mode_target_altitude_m;

    if ((now_ms - state->mode_entry_ms) < FC_LAND_SETTLE_MS) {
        return false;
    }

    return (state->estimate.altitude_m <= (ground_altitude_m + FC_LAND_ALTITUDE_MARGIN_M)) &&
           (abs_float(state->estimate.vertical_speed_mps) <= FC_LAND_VERTICAL_SPEED_MAX_MPS);
}

static void commander_update_state(flight_state_t *state)
{
    const uint32_t errors = state->error_flags;

    if (errors != 0U) {
        if (state->armed && only_rc_failsafe(errors) && state->estimate.altitude_valid) {
            enter_mode(state, FLIGHT_MODE_LAND);
        } else {
            enter_mode(state, state->armed ? FLIGHT_MODE_FAILSAFE : FLIGHT_MODE_LOCKED);
        }
        return;
    }

    switch (state->mode) {
    case FLIGHT_MODE_LOCKED:
        enter_mode(state, FLIGHT_MODE_STANDBY);
        break;

    case FLIGHT_MODE_STANDBY:
        state->armed = false;
        if (controls_are_safe_to_arm(state) &&
            attitude_is_safe_to_arm(state) &&
            state->estimate.altitude_valid) {
            enter_mode(state, FLIGHT_MODE_ARMED_IDLE);
        }
        break;

    case FLIGHT_MODE_ARMED_IDLE:
        if (!state->rc.arm_switch) {
            enter_mode(state, FLIGHT_MODE_STANDBY);
        } else if (takeoff_requested(state) && state->estimate.altitude_valid) {
            enter_mode(state, FLIGHT_MODE_TAKEOFF);
        }
        break;

    case FLIGHT_MODE_TAKEOFF:
        if (!state->rc.arm_switch) {
            enter_mode(state, FLIGHT_MODE_LAND);
        } else if (takeoff_complete(state)) {
            enter_mode(state, FLIGHT_MODE_ALT_HOLD);
        }
        break;

    case FLIGHT_MODE_ALT_HOLD:
    case FLIGHT_MODE_MISSION:
        if (!state->rc.arm_switch) {
            enter_mode(state, FLIGHT_MODE_LAND);
        }
        break;

    case FLIGHT_MODE_LAND:
        if (land_complete(state) || (!state->rc.arm_switch && !state->estimate.altitude_valid)) {
            enter_mode(state, FLIGHT_MODE_STANDBY);
        }
        break;

    case FLIGHT_MODE_FAILSAFE:
        if (!state->rc.arm_switch) {
            enter_mode(state, FLIGHT_MODE_LOCKED);
        }
        break;

    case FLIGHT_MODE_MANUAL_STAB:
    default:
        enter_mode(state, FLIGHT_MODE_STANDBY);
        break;
    }
}

void CommanderTask(void *argument)
{
    (void)argument;
    g_dbg_boot.startup_phase = 103U;
    g_dbg_boot.task_entry_mask |= (1UL << TASK_INDEX_COMMANDER);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = task_period_ticks(FC_COMMANDER_TASK_HZ);
    flight_state_t *state = app_state();
    uint8_t led = 0U;

    while (1) {
        const TickType_t loop_start = xTaskGetTickCount();
        safety_update(state);
        commander_update_state(state);
        mission_update(state);

        led ^= 1U;
        bsp_board_led_set(led);

        task_record_heartbeat(TASK_INDEX_COMMANDER, loop_start);
        vTaskDelayUntil(&last_wake, period);
    }
}
