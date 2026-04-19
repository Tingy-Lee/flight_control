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
    app_state_init();

    (void)imu_init();
    (void)barometer_init();
    (void)gps_init();
    (void)rc_input_init();

    attitude_estimator_init(&g_state.estimate);
    flight_controller_init();
    safety_init();
    mission_init();
    logger_init();

    bsp_pwm_motors_set_all_us(FC_MOTOR_PWM_MIN_US);
    tasks_create_all();

    vTaskStartScheduler();

    while (1) {
    }
}

void vApplicationMallocFailedHook(void)
{
    bsp_pwm_motors_set_all_us(FC_MOTOR_PWM_MIN_US);
    taskDISABLE_INTERRUPTS();
    while (1) {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    bsp_pwm_motors_set_all_us(FC_MOTOR_PWM_MIN_US);
    taskDISABLE_INTERRUPTS();
    while (1) {
    }
}
