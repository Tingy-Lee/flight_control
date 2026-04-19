#include "tasks/Inc/tasks.h"

#include <stdio.h>

#define TASK_PRIO_SENSOR       8
#define TASK_PRIO_CONTROL      9
#define TASK_PRIO_COMMANDER    6
#define TASK_PRIO_LOGGER       3

#define TASK_STACK_SENSOR      512
#define TASK_STACK_CONTROL     768
#define TASK_STACK_COMMANDER   512
#define TASK_STACK_LOGGER      512

TaskHandle_t sensorTaskHandle;
TaskHandle_t controlTaskHandle;
TaskHandle_t commanderTaskHandle;
TaskHandle_t loggerTaskHandle;

TickType_t task_period_ticks(uint32_t hz)
{
    if (hz == 0U) {
        return pdMS_TO_TICKS(1000U);
    }

    return pdMS_TO_TICKS(1000U / hz);
}

void tasks_create_all(void)
{
    if ((xTaskCreate(SensorTask, "sensor", TASK_STACK_SENSOR, 0, TASK_PRIO_SENSOR, &sensorTaskHandle) != pdPASS) ||
        (xTaskCreate(ControlTask, "control", TASK_STACK_CONTROL, 0, TASK_PRIO_CONTROL, &controlTaskHandle) != pdPASS) ||
        (xTaskCreate(CommanderTask, "commander", TASK_STACK_COMMANDER, 0, TASK_PRIO_COMMANDER, &commanderTaskHandle) != pdPASS) ||
        (xTaskCreate(LoggerTask, "logger", TASK_STACK_LOGGER, 0, TASK_PRIO_LOGGER, &loggerTaskHandle) != pdPASS)) {
        printf("task create failed\r\n");
        while (1) {
        }
    }
}
