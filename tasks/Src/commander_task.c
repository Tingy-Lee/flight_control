#include "tasks/Inc/tasks.h"

#include "app/app.h"
#include "app/app_config.h"
#include "bsp/bsp_board.h"
#include "modules/navigation/mission.h"
#include "modules/safety/safety.h"

extern volatile uint32_t g_dbg_startup_phase;
extern volatile uint32_t g_dbg_task_entry_mask;

void CommanderTask(void *argument)
{
    (void)argument;
    g_dbg_startup_phase = 103U;
    g_dbg_task_entry_mask |= (1UL << 2);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = task_period_ticks(FC_COMMANDER_TASK_HZ);
    flight_state_t *state = app_state();
    uint8_t led = 0U;

    while (1) {
        const TickType_t loop_start = xTaskGetTickCount();
        safety_update(state);
        mission_update(state);

        led ^= 1U;
        bsp_board_led_set(led);

        task_record_heartbeat(TASK_INDEX_COMMANDER, loop_start);
        vTaskDelayUntil(&last_wake, period);
    }
}
