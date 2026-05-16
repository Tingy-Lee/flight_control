#include "tasks/Inc/tasks.h"

#include "app/app.h"
#include "app/app_config.h"
#include "debug_diagnostics.h"
#include "modules/comm/logger.h"

void LoggerTask(void *argument)
{
    (void)argument;
    g_dbg_boot.startup_phase = 104U;
    g_dbg_boot.task_entry_mask |= (1UL << TASK_INDEX_LOGGER);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = task_period_ticks(FC_LOGGER_TASK_HZ);
    flight_state_t *state = app_state();
    uint8_t decimator = 0U;

    while (1) {
        const TickType_t loop_start = xTaskGetTickCount();
#if FC_ENABLE_RUNTIME_LOGGER
        if (++decimator >= 5U) {
            decimator = 0U;
            logger_print_state(state);
        }
#else
        (void)state;
        (void)decimator;
#endif

        task_record_heartbeat(TASK_INDEX_LOGGER, loop_start);
        vTaskDelayUntil(&last_wake, period);
    }
}
