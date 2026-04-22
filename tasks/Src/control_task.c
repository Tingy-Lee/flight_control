#include "tasks/Inc/tasks.h"

#include "app/app.h"
#include "app/app_config.h"
#include "bsp/bsp_pwm.h"
#include "modules/control/flight_controller.h"
#include "modules/estimator/attitude_estimator.h"

extern volatile uint32_t g_dbg_startup_phase;
extern volatile uint32_t g_dbg_task_entry_mask;

void ControlTask(void *argument)
{
    (void)argument;
    g_dbg_startup_phase = 102U;
    g_dbg_task_entry_mask |= (1UL << 1);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = task_period_ticks(FC_CONTROL_TASK_HZ);
    const float dt_s = 1.0f / (float)FC_CONTROL_TASK_HZ;
    flight_state_t *state = app_state();

    while (1) {
        const TickType_t loop_start = xTaskGetTickCount();
        attitude_estimator_update(&state->estimate, &state->imu, &state->baro, dt_s);
        flight_controller_update(state, dt_s);
        bsp_pwm_motors_write(state->motors.motor_us);

        task_record_heartbeat(TASK_INDEX_CONTROL, loop_start);
        vTaskDelayUntil(&last_wake, period);
    }
}
