#include "app/app.h"
#include "app/app_config.h"
#include "tasks/Inc/tasks.h"
#include "bsp/bsp_pwm.h"
#include "drivers/sensors/barometer.h"
#include "drivers/sensors/gps.h"
#include "drivers/sensors/imu.h"
#include "drivers/sensors/rc_input.h"
#include "modules/comm/logger.h"
#include "modules/control/flight_controller.h"
#include "modules/estimator/attitude_estimator.h"
#include "modules/navigation/mission.h"
#include "modules/safety/safety.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static flight_state_t g_state;
extern volatile uint32_t g_dbg_startup_phase;

volatile const char *g_dbg_assert_file;
volatile uint32_t g_dbg_assert_line;
volatile uint32_t g_dbg_assert_count;
volatile uint32_t g_dbg_assert_heap_free;
volatile uint32_t g_dbg_assert_heap_min_free;
volatile uint32_t g_dbg_assert_return_address;
volatile uint32_t g_dbg_assert_scheduler_state;
volatile uint32_t g_dbg_assert_tick;
volatile void *g_dbg_assert_current_task;
volatile uint32_t g_dbg_fault_reason;
volatile void *g_dbg_fault_task;
volatile const char *g_dbg_fault_task_name;
volatile uint32_t g_dbg_heap_before_scheduler;

#define DBG_FAULT_ASSERT          0xA55E0001UL
#define DBG_FAULT_MALLOC_FAILED   0xA55E0002UL
#define DBG_FAULT_STACK_OVERFLOW  0xA55E0003UL

flight_state_t *app_state(void)
{
    return &g_state;
}

static void app_state_init(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state.mode = FLIGHT_MODE_LOCKED;
    g_state.armed = false;
    for (uint8_t i = 0; i < FC_MOTOR_COUNT; i++) {
        g_state.motors.motor_us[i] = FC_MOTOR_PWM_MIN_US;
    }
}

void app_start(void)
{
    g_dbg_startup_phase = 30U;
    app_state_init();

    g_dbg_startup_phase = 31U;
    (void)imu_init();
    g_dbg_startup_phase = 32U;
    (void)barometer_init();
    g_dbg_startup_phase = 33U;
    (void)gps_init();
    g_dbg_startup_phase = 34U;
    (void)rc_input_init();

    g_dbg_startup_phase = 40U;
    attitude_estimator_init(&g_state.estimate);
    flight_controller_init();
    safety_init();
    mission_init();
    logger_init();

    bsp_pwm_motors_set_all_us(FC_MOTOR_PWM_MIN_US);
    g_dbg_startup_phase = 50U;
    tasks_create_all();
    g_dbg_heap_before_scheduler = (uint32_t)xPortGetFreeHeapSize();
    g_dbg_startup_phase = 60U;

    vTaskStartScheduler();
    g_dbg_startup_phase = 200U;

    while (1) {
    }
}

void vApplicationMallocFailedHook(void)
{
    g_dbg_fault_reason = DBG_FAULT_MALLOC_FAILED;
    g_dbg_assert_heap_free = (uint32_t)xPortGetFreeHeapSize();
    g_dbg_assert_heap_min_free = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    bsp_pwm_motors_set_all_us(FC_MOTOR_PWM_MIN_US);
    taskDISABLE_INTERRUPTS();
    while (1) {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    g_dbg_fault_reason = DBG_FAULT_STACK_OVERFLOW;
    g_dbg_fault_task = (void *)task;
    g_dbg_fault_task_name = task_name;
    g_dbg_assert_heap_free = (uint32_t)xPortGetFreeHeapSize();
    g_dbg_assert_heap_min_free = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    bsp_pwm_motors_set_all_us(FC_MOTOR_PWM_MIN_US);
    taskDISABLE_INTERRUPTS();
    while (1) {
    }
}

void vApplicationAssertFailed(const char *file, int line)
{
    BaseType_t scheduler_state = xTaskGetSchedulerState();

    g_dbg_fault_reason = DBG_FAULT_ASSERT;
    g_dbg_assert_file = file;
    g_dbg_assert_line = (uint32_t)line;
    g_dbg_assert_count++;
    g_dbg_assert_heap_free = (uint32_t)xPortGetFreeHeapSize();
    g_dbg_assert_heap_min_free = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    g_dbg_assert_return_address = (uint32_t)(uintptr_t)__builtin_return_address(0);
    g_dbg_assert_scheduler_state = (uint32_t)scheduler_state;
    g_dbg_assert_tick = (scheduler_state != taskSCHEDULER_NOT_STARTED) ?
                        (uint32_t)xTaskGetTickCount() :
                        0U;
    g_dbg_assert_current_task = (scheduler_state != taskSCHEDULER_NOT_STARTED) ?
                                (void *)xTaskGetCurrentTaskHandle() :
                                (void *)0;

    bsp_pwm_motors_set_all_us(FC_MOTOR_PWM_MIN_US);
    taskDISABLE_INTERRUPTS();
    while (1) {
    }
}
