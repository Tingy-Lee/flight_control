#include "tasks/Inc/tasks.h"

#include "app/app.h"
#include "app/app_config.h"
#include "bsp/bsp_uart.h"
#include "debug_diagnostics.h"
#include "drivers/sensors/barometer.h"
#include "drivers/sensors/gps.h"
#include "drivers/sensors/imu.h"
#include "drivers/sensors/rc_input.h"

void SensorTask(void *argument)
{
    (void)argument;
    g_dbg_boot.startup_phase = 101U;
    g_dbg_boot.task_entry_mask |= (1UL << TASK_INDEX_SENSOR);
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = task_period_ticks(FC_SENSOR_TASK_HZ);
    flight_state_t *state = app_state();

#if FC_ENABLE_IMU_UART
    bsp_uart_imu_start_rx_irq();
#endif
#if FC_ENABLE_GPS_USART2
    bsp_uart_gps_start_rx_irq();
#endif

    while (1) {
        const TickType_t loop_start = xTaskGetTickCount();
#if FC_ENABLE_IMU_UART
        bsp_uart_debug_sample_imu_rx();
#endif
#if FC_ENABLE_RC_USART3
        bsp_uart_debug_sample_rc_rx();
#endif
        (void)imu_read(&state->imu);
        (void)barometer_read(&state->baro);
        (void)gps_poll(&state->gps);
        (void)rc_input_poll(&state->rc);

        task_record_heartbeat(TASK_INDEX_SENSOR, loop_start);
        vTaskDelayUntil(&last_wake, period);
    }
}
