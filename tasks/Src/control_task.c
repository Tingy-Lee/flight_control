#include "tasks/Inc/tasks.h"

#include "app/app.h"
#include "app/app_config.h"
#include "bsp/bsp_board.h"
#include "bsp/bsp_pwm.h"
#include "debug_diagnostics.h"
#include "modules/control/flight_controller.h"
#include "modules/estimator/attitude_estimator.h"
#include <stdio.h>

#if FC_ENABLE_MOTOR_PWM_TEST
static uint16_t motor_test_clamp_us(uint16_t pulse_us)
{
    if (pulse_us < FC_MOTOR_PWM_MIN_US) {
        return FC_MOTOR_PWM_MIN_US;
    }
    if (pulse_us > FC_MOTOR_PWM_MAX_US) {
        return FC_MOTOR_PWM_MAX_US;
    }

    return pulse_us;
}

static void motor_test_update(flight_state_t *state)
{
    uint16_t pulse_us = FC_MOTOR_PWM_MIN_US;

    if (state == 0) {
        return;
    }

    if (state->rc.healthy && state->rc.arm_switch) {
        pulse_us = motor_test_clamp_us(state->rc.throttle_us);
    }

    for (uint8_t i = 0U; i < FC_MOTOR_COUNT; i++) {
#if FC_MOTOR_TEST_SELECTED_INDEX < FC_MOTOR_COUNT
        state->motors.motor_us[i] = (i == FC_MOTOR_TEST_SELECTED_INDEX) ? pulse_us : FC_MOTOR_PWM_MIN_US;
#else
        state->motors.motor_us[i] = pulse_us;
#endif
    }

    state->setpoint.throttle_norm =
        (float)(pulse_us - FC_MOTOR_PWM_MIN_US) /
        (float)(FC_MOTOR_PWM_MAX_US - FC_MOTOR_PWM_MIN_US);
}

#endif

static void control_set_motors_min(flight_state_t *state)
{
    if (state == 0) {
        return;
    }

    for (uint8_t i = 0U; i < FC_MOTOR_COUNT; i++) {
        state->motors.motor_us[i] = FC_MOTOR_PWM_MIN_US;
    }
    state->setpoint.throttle_norm = 0.0f;
    state->setpoint.roll_cmd = 0.0f;
    state->setpoint.pitch_cmd = 0.0f;
    state->setpoint.yaw_cmd = 0.0f;
}

static bool control_motor_kill_requested(const flight_state_t *state)
{
    if (state == 0) {
        return true;
    }

    if ((!state->rc.healthy) || state->rc.failsafe || state->rc.frame_lost) {
        return true;
    }

    if (!state->rc.arm_switch) {
        return true;
    }

    return state->rc.throttle_us <= FC_ARM_THROTTLE_MAX_US;
}

#if FC_ENABLE_MOTOR_TEST_TELEMETRY
static int32_t telemetry_milli(float value)
{
    return (int32_t)(value * 1000.0f);
}

static uint16_t telemetry_channel_us(const flight_state_t *state, uint8_t channel)
{
    if ((state == 0) || (channel >= FC_RC_INPUT_CHANNEL_COUNT)) {
        return 0U;
    }

    return state->rc.channels_us[channel];
}

static void motor_telemetry_print(const flight_state_t *state)
{
    static uint32_t last_print_ms;
    const uint32_t now_ms = (uint32_t)xTaskGetTickCount();
    const uint32_t period_ms = 1000U / FC_MOTOR_TEST_TELEMETRY_HZ;

    if ((now_ms - last_print_ms) < period_ms) {
        return;
    }
    last_print_ms = now_ms;

    printf("PWM,%lu"
           ",%u,%u,%u,%u,%u,%u,%u"
           ",%u,%u,%u,%u"
           ",%u,%u,%u,%u"
           ",%u,%u,%u,%u"
           ",%ld,%ld,%ld,%ld,%ld\r\n",
           (unsigned long)now_ms,
           (unsigned)state->mode,
           (unsigned)state->armed,
           (unsigned)state->rc.arm_switch,
           (unsigned)state->rc.healthy,
           (unsigned)state->rc.failsafe,
           (unsigned)state->rc.frame_lost,
           (unsigned)state->rc.throttle_us,
           (unsigned)telemetry_channel_us(state, FC_RC_CHANNEL_ARM),
           (unsigned)telemetry_channel_us(state, 2U),
           (unsigned)telemetry_channel_us(state, 4U),
           (unsigned)telemetry_channel_us(state, 6U),
           (unsigned)g_dbg_pwm.tim1_ccr[0],
           (unsigned)g_dbg_pwm.tim1_ccr[1],
           (unsigned)g_dbg_pwm.tim1_ccr[2],
           (unsigned)g_dbg_pwm.tim1_ccr[3],
           (unsigned)g_dbg_pwm.requested_us[0],
           (unsigned)g_dbg_pwm.requested_us[1],
           (unsigned)g_dbg_pwm.requested_us[2],
           (unsigned)g_dbg_pwm.requested_us[3],
           (long)telemetry_milli(state->setpoint.throttle_norm),
           (long)telemetry_milli(state->setpoint.roll_cmd),
           (long)telemetry_milli(state->setpoint.pitch_cmd),
           (long)telemetry_milli(state->setpoint.yaw_cmd),
           (long)telemetry_milli(state->estimate.altitude_m));
}
#endif

void ControlTask(void *argument)
{
    (void)argument;
    g_dbg_boot.startup_phase = 102U;
    g_dbg_boot.task_entry_mask |= (1UL << TASK_INDEX_CONTROL);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = task_period_ticks(FC_CONTROL_TASK_HZ);
    const float dt_s = 1.0f / (float)FC_CONTROL_TASK_HZ;
    flight_state_t *state = app_state();

    while (1) {
        const TickType_t loop_start = xTaskGetTickCount();
        attitude_estimator_update(&state->estimate, &state->imu, &state->baro, dt_s);
#if FC_ENABLE_MOTOR_PWM_TEST
        motor_test_update(state);
#else
        flight_controller_update(state, dt_s);
#endif
        if (control_motor_kill_requested(state)) {
            control_set_motors_min(state);
        }
        bsp_pwm_motors_write(state->motors.motor_us);
        {
            static uint32_t last_ms;
            const uint32_t now_ms = bsp_board_millis();
            if ((now_ms - last_ms) >= 100U) {
                last_ms = now_ms;
                const int32_t p_dif = ((int32_t)state->motors.motor_us[0] + (int32_t)state->motors.motor_us[1]) -
                                     ((int32_t)state->motors.motor_us[2] + (int32_t)state->motors.motor_us[3]);
                const int32_t r_dif = ((int32_t)state->motors.motor_us[1] + (int32_t)state->motors.motor_us[2]) -
                                     ((int32_t)state->motors.motor_us[0] + (int32_t)state->motors.motor_us[3]);
                const int32_t y_dif = ((int32_t)state->motors.motor_us[1] + (int32_t)state->motors.motor_us[3]) -
                                     ((int32_t)state->motors.motor_us[0] + (int32_t)state->motors.motor_us[2]);
                printf("M:%u %u %u %u  dP:%+d dR:%+d  [%u] thr=%u rch=%u tgt=%u\r\n",
                       (unsigned)state->motors.motor_us[0],
                       (unsigned)state->motors.motor_us[1],
                       (unsigned)state->motors.motor_us[2],
                       (unsigned)state->motors.motor_us[3],
                       (int)p_dif, (int)r_dif,
                       (unsigned)state->mode,
                       (unsigned)state->rc.throttle_us,
                       (unsigned)(state->rc.healthy ? 1 : 0),
                       (unsigned)(state->target.valid ? 1 : 0));
            }
        }
#if FC_ENABLE_MOTOR_TEST_TELEMETRY
        motor_telemetry_print(state);
#endif
     //  bsp_pwm_motors_debug();

        task_record_heartbeat(TASK_INDEX_CONTROL, loop_start);
        vTaskDelayUntil(&last_wake, period);
    }
}
