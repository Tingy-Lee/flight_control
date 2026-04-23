#include "tasks/Inc/tasks.h"

#include "app/app_config.h"
#include "debug_diagnostics.h"

#define TASK_PRIO_SENSOR       8
#define TASK_PRIO_CONTROL      9
#define TASK_PRIO_COMMANDER    6
#define TASK_PRIO_LOGGER       3

#define TASK_STACK_SENSOR      1024
#define TASK_STACK_CONTROL     1024
#define TASK_STACK_COMMANDER   768
#define TASK_STACK_LOGGER      768

#define TASK_STACK_SAMPLE_MASK 0x7FU

TaskHandle_t sensorTaskHandle;
TaskHandle_t controlTaskHandle;
TaskHandle_t commanderTaskHandle;
TaskHandle_t loggerTaskHandle;

static TaskHandle_t task_handle_for_index(uint8_t index)
{
    switch (index) {
    case TASK_INDEX_SENSOR:
        return sensorTaskHandle;
    case TASK_INDEX_CONTROL:
        return controlTaskHandle;
    case TASK_INDEX_COMMANDER:
        return commanderTaskHandle;
    case TASK_INDEX_LOGGER:
        return loggerTaskHandle;
    default:
        return 0;
    }
}

TickType_t task_period_ticks(uint32_t hz)
{
    if (hz == 0U) {
        return pdMS_TO_TICKS(1000U);
    }

    TickType_t ticks = (TickType_t)(((uint32_t)configTICK_RATE_HZ + hz - 1U) / hz);
    if (ticks == 0U) {
        ticks = 1U;
    }

    return ticks;
}

void task_record_heartbeat(uint8_t index, TickType_t loop_start_tick)
{
    if (index >= TASK_INDEX_COUNT) {
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    const uint32_t elapsed = (uint32_t)(now - loop_start_tick);
    const uint32_t loop_count = g_dbg_tasks.loop_count[index] + 1U;

    g_dbg_tasks.loop_count[index] = loop_count;
    g_dbg_tasks.last_tick[index] = (uint32_t)now;
    if (elapsed > g_dbg_tasks.max_exec_ticks[index]) {
        g_dbg_tasks.max_exec_ticks[index] = elapsed;
    }

    if ((loop_count & TASK_STACK_SAMPLE_MASK) == 0U) {
        TaskHandle_t handle = task_handle_for_index(index);
        if (handle != 0) {
            g_dbg_tasks.stack_free_words[index] = uxTaskGetStackHighWaterMark(handle);
        }

        g_dbg_tasks.runtime_heap_free = (uint32_t)xPortGetFreeHeapSize();
        g_dbg_tasks.runtime_heap_min_free = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    }
}

void tasks_create_all(void)
{
    g_dbg_tasks.heap_before_create = (uint32_t)xPortGetFreeHeapSize();
    g_dbg_tasks.period_ticks[TASK_INDEX_SENSOR] = (uint32_t)task_period_ticks(FC_SENSOR_TASK_HZ);
    g_dbg_tasks.period_ticks[TASK_INDEX_CONTROL] = (uint32_t)task_period_ticks(FC_CONTROL_TASK_HZ);
    g_dbg_tasks.period_ticks[TASK_INDEX_COMMANDER] = (uint32_t)task_period_ticks(FC_COMMANDER_TASK_HZ);
    g_dbg_tasks.period_ticks[TASK_INDEX_LOGGER] = (uint32_t)task_period_ticks(FC_LOGGER_TASK_HZ);

    g_dbg_tasks.create_result[0] = xTaskCreate(SensorTask, "sensor", TASK_STACK_SENSOR, 0, TASK_PRIO_SENSOR, &sensorTaskHandle);
    g_dbg_tasks.heap_after_create[0] = (uint32_t)xPortGetFreeHeapSize();
    configASSERT(g_dbg_tasks.create_result[0] == pdPASS);

    g_dbg_tasks.create_result[1] = xTaskCreate(ControlTask, "control", TASK_STACK_CONTROL, 0, TASK_PRIO_CONTROL, &controlTaskHandle);
    g_dbg_tasks.heap_after_create[1] = (uint32_t)xPortGetFreeHeapSize();
    configASSERT(g_dbg_tasks.create_result[1] == pdPASS);

    g_dbg_tasks.create_result[2] = xTaskCreate(CommanderTask, "commander", TASK_STACK_COMMANDER, 0, TASK_PRIO_COMMANDER, &commanderTaskHandle);
    g_dbg_tasks.heap_after_create[2] = (uint32_t)xPortGetFreeHeapSize();
    configASSERT(g_dbg_tasks.create_result[2] == pdPASS);

    g_dbg_tasks.create_result[3] = xTaskCreate(LoggerTask, "logger", TASK_STACK_LOGGER, 0, TASK_PRIO_LOGGER, &loggerTaskHandle);
    g_dbg_tasks.heap_after_create[3] = (uint32_t)xPortGetFreeHeapSize();
    configASSERT(g_dbg_tasks.create_result[3] == pdPASS);
}
