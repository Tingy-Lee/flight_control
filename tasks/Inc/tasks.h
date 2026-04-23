#ifndef TASKS_H
#define TASKS_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t sensorTaskHandle;
extern TaskHandle_t controlTaskHandle;
extern TaskHandle_t commanderTaskHandle;
extern TaskHandle_t loggerTaskHandle;

enum {
    TASK_INDEX_SENSOR = 0,
    TASK_INDEX_CONTROL,
    TASK_INDEX_COMMANDER,
    TASK_INDEX_LOGGER,
    TASK_INDEX_COUNT
};

void tasks_create_all(void);
TickType_t task_period_ticks(uint32_t hz);
void task_record_heartbeat(uint8_t index, TickType_t loop_start_tick);

void SensorTask(void *argument);
void ControlTask(void *argument);
void CommanderTask(void *argument);
void LoggerTask(void *argument);

#endif
