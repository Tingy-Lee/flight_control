#ifndef TASKS_H
#define TASKS_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t sensorTaskHandle;
extern TaskHandle_t controlTaskHandle;
extern TaskHandle_t commanderTaskHandle;
extern TaskHandle_t loggerTaskHandle;

void tasks_create_all(void);
TickType_t task_period_ticks(uint32_t hz);

void SensorTask(void *argument);
void ControlTask(void *argument);
void CommanderTask(void *argument);
void LoggerTask(void *argument);

#endif
