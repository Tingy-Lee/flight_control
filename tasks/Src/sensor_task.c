#include "tasks/Inc/tasks.h"

#include "app/app.h"
#include "app/app_config.h"
#include "drivers/sensors/barometer.h"
#include "drivers/sensors/gps.h"
#include "drivers/sensors/imu.h"
#include "drivers/sensors/rc_input.h"

void SensorTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = task_period_ticks(FC_SENSOR_TASK_HZ);
    flight_state_t *state = app_state();

    while (1) {
        (void)imu_read(&state->imu);
        (void)barometer_read(&state->baro);
        (void)gps_poll(&state->gps);
        (void)rc_input_poll(&state->rc);

        vTaskDelayUntil(&last_wake, period);
    }
}
