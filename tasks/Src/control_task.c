#include "tasks/Inc/tasks.h"

#include "app/app.h"
#include "app/app_config.h"
#include "bsp/bsp_pwm.h"
#include "modules/control/flight_controller.h"
#include "modules/estimator/attitude_estimator.h"

void ControlTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = task_period_ticks(FC_CONTROL_TASK_HZ);
    const float dt_s = 1.0f / (float)FC_CONTROL_TASK_HZ;
    flight_state_t *state = app_state();

    while (1) {
        attitude_estimator_update(&state->estimate, &state->imu, &state->baro, dt_s);
        flight_controller_update(state, dt_s);
        bsp_pwm_motors_write(state->motors.motor_us);

        vTaskDelayUntil(&last_wake, period);
    }
}
