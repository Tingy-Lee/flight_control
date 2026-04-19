#include "tasks/Inc/tasks.h"

#include "app/app.h"
#include "app/app_config.h"
#include "modules/comm/logger.h"

void LoggerTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = task_period_ticks(FC_LOGGER_TASK_HZ);
    flight_state_t *state = app_state();
    uint8_t decimator = 0U;

    while (1) {
        if (++decimator >= 5U) {
            decimator = 0U;
            logger_print_state(state);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
