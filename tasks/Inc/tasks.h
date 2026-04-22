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

extern volatile uint32_t g_dbg_task_loop_count[TASK_INDEX_COUNT];
extern volatile uint32_t g_dbg_task_last_tick[TASK_INDEX_COUNT];
extern volatile uint32_t g_dbg_task_max_exec_ticks[TASK_INDEX_COUNT];
extern volatile UBaseType_t g_dbg_task_stack_free_words[TASK_INDEX_COUNT];
extern volatile uint32_t g_dbg_task_period_ticks[TASK_INDEX_COUNT];
extern volatile uint32_t g_dbg_runtime_heap_free;
extern volatile uint32_t g_dbg_runtime_heap_min_free;

void tasks_create_all(void);
TickType_t task_period_ticks(uint32_t hz);
void task_record_heartbeat(uint8_t index, TickType_t loop_start_tick);

void SensorTask(void *argument);
void ControlTask(void *argument);
void CommanderTask(void *argument);
void LoggerTask(void *argument);

#endif
