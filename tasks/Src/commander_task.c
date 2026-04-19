#include "tasks/Inc/tasks.h"

#include "app/app.h"
#include "app/app_config.h"
#include "bsp/bsp_board.h"
#include "modules/navigation/mission.h"
#include "modules/safety/safety.h"

void CommanderTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = task_period_ticks(FC_COMMANDER_TASK_HZ);
    flight_state_t *state = app_state();
    uint8_t led = 0U;

    while (1) {
        safety_update(state);
        mission_update(state);

        led ^= 1U;
        bsp_board_led_set(led);

        vTaskDelayUntil(&last_wake, period);
    }
}
