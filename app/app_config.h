#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* Project identity */
#define FC_PROJECT_NAME                 "CH32H417 FlightControl"
#define FC_PROJECT_VERSION              "0.1.0"
#define FC_AIRFRAME_NAME                "X-Quad / EMAX MT2217"

/* Bring-up safety: keep arming disabled until RC and sensor health are real. */
#define FC_ALLOW_ARMING                 0

/* Scheduler rates */
#define FC_SENSOR_TASK_HZ               500U
#define FC_CONTROL_TASK_HZ              500U
#define FC_COMMANDER_TASK_HZ            50U
#define FC_LOGGER_TASK_HZ               10U

/* Motor PWM for CH32V203 ESC command input. 400 Hz is suitable for the
 * custom ESC decoder target; change to 50 Hz if testing hobby ESCs first.
 */
#define FC_MOTOR_PWM_HZ                 400U
#define FC_MOTOR_PWM_MIN_US             1000U
#define FC_MOTOR_PWM_IDLE_US            1050U
#define FC_MOTOR_PWM_MAX_US             2000U
#define FC_MOTOR_COUNT                  4U

/* Initial bench limit for EMAX MT2217 bring-up. Raise only after prop-off
 * output direction checks and tethered low-throttle tests.
 */
#define FC_MOTOR_BRINGUP_LIMIT_US       1200U

/* Board-level feature switches. I2C2 uses PC0/PC1 in WCH examples. SPI2 also
 * uses PC1 as MOSI, so keep SPI disabled until pins are separated on the
 * carrier wiring or custom PCB.
 *
 * The ten-axis IMU is currently used through its UART auto-report protocol on
 * USART4: PF4 TX, PF3 RX.
 */
#define FC_ENABLE_I2C2_SENSORS          0
#define FC_ENABLE_IMU_UART              1
#define FC_ENABLE_SPI2_IMU              0
#define FC_ENABLE_GPS_USART2            1
#define FC_ENABLE_RC_USART3             1
#define FC_ENABLE_BATTERY_ADC           1
#define FC_ENABLE_RUNTIME_LOGGER        0

/* Sensor defaults */
#define FC_SEA_LEVEL_PRESSURE_PA        101325.0f
#define FC_GRAVITY_MPS2                 9.80665f
#define FC_IMU_UART_BAUD                115200U
#define FC_IMU_UART_OUTPUT_HZ           50U
#define FC_IMU_UART_STALE_TIMEOUT_MS    500U

#endif
