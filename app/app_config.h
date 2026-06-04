#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* Project identity */
#define FC_PROJECT_NAME                 "CH32H417 FlightControl"
#define FC_PROJECT_VERSION              "0.1.0"
#define FC_AIRFRAME_NAME                "X-Quad / EMAX MT2217"

/* Bring-up safety: keep arming disabled until RC and sensor health are real. */
#define FC_ALLOW_ARMING                 1

/* Scheduler rates */
#define FC_SENSOR_TASK_HZ               250U
#define FC_DECISION_TASK_HZ             50U
#define FC_CONTROL_TASK_HZ              500U
#define FC_COMMANDER_TASK_HZ            50U
#define FC_LOGGER_TASK_HZ               10U

/* Motor PWM for hobby ESC command input. Use 50 Hz while bringing up BLHeli
 * ESCs, then raise only after checking the target ESC accepts the faster rate.
 * SPIN_MIN keeps motors above the observed low-throttle stutter region during
 * armed flight; tune it with propellers removed.
 */
#define FC_MOTOR_PWM_HZ                 50U
#define FC_MOTOR_PWM_MIN_US             1000U
#define FC_MOTOR_PWM_IDLE_US            1050U
#define FC_MOTOR_PWM_SPIN_MIN_US        1180U
#define FC_MOTOR_PWM_MAX_US             2000U
#define FC_MOTOR_COUNT                  4U

/* Initial bench limit for EMAX MT2217 bring-up. Keep propellers removed while
 * this is raised to the normal hobby PWM range.
 */
#define FC_MOTOR_BRINGUP_LIMIT_US       2000U

/* Bench-only PWM passthrough test.
 * When enabled, CH7 gates motor output and CH3 is written directly as PWM.
 * Set FC_MOTOR_TEST_SELECTED_INDEX to 0..3 for one motor, or 255 for all.
 */
#define FC_ENABLE_MOTOR_PWM_TEST        0
#define FC_MOTOR_TEST_SELECTED_INDEX    255U
#define FC_ENABLE_MOTOR_TEST_TELEMETRY  0
#define FC_MOTOR_TEST_TELEMETRY_HZ      20U

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
#define FC_IMU_UART_OUTPUT_HZ           100U
/* Yahboom 10-axis IMU frames were measured near 80 ms nominal with occasional
 * 245 ms motion gaps on the current UART setup. Keep attitude fresh checks
 * tighter than barometer checks, but allow more than two nominal frame periods.
 */
#define FC_IMU_UART_STALE_TIMEOUT_MS    350U
#define FC_IMU_BARO_STALE_TIMEOUT_MS    600U
#define FC_GPS_UART_BAUD                9600U
#define FC_GPS_STALE_TIMEOUT_MS         2000U

/* FlySky iBUS defaults for FS-i6X + FS-iA10B. Channel numbering here is
 * 0-based, so CH1 -> 0, CH2 -> 1, and so on.
 */
#define FC_RC_IBUS_BAUD                 115200U
#define FC_RC_IBUS_STALE_TIMEOUT_MS     100U
#define FC_RC_INPUT_CHANNEL_COUNT       10U
#define FC_RC_CHANNEL_ROLL              0U
#define FC_RC_CHANNEL_PITCH             1U
#define FC_RC_CHANNEL_THROTTLE          2U
#define FC_RC_CHANNEL_YAW               3U
#define FC_RC_CHANNEL_HOVER_HEIGHT      4U
/* CH7 is a two-position switch on FS-i6X/iA10B. FlySky iBUS channel numbers
 * are 0-based here, so CH7 maps to index 6. Leave ACTIVE_HIGH enabled when
 * the armed/on switch position reads near 2000 us; set it to 0 if armed/on
 * reads near 1000 us.
 */
#define FC_RC_CHANNEL_ARM               6U
#define FC_RC_CHANNEL_HEIGHT_SWITCH     9U
#define FC_RC_SWITCH_ACTIVE_THRESHOLD_US 1600U
#define FC_RC_ARM_ACTIVE_HIGH           1
#define FC_RC_ARM_THRESHOLD_US          1600U
#define FC_RC_ARM_LOW_THRESHOLD_US      1400U
#define FC_ARM_THROTTLE_MAX_US          (FC_MOTOR_PWM_MIN_US + 30U)

/* RC decision mapping. Roll/pitch are attitude targets; CH3 is the throttle
 * input used for arming checks and takeoff trigger logic; CH4 is a body-axis
 * yaw angular-rate target; CH5 is interpreted as a hover-height change rate
 * and integrated into a bounded height delta.
 */
#define FC_RC_PWM_CENTER_US             1500U
#define FC_RC_PWM_HALF_RANGE_US         500U
#define FC_RC_PWM_DEADZONE_US           20U
#define FC_RC_HOVER_DEADBAND_US         100U
#define FC_TARGET_MAX_ROLL_RAD          0.34906585f
#define FC_TARGET_MAX_PITCH_RAD         0.34906585f
#define FC_TARGET_MAX_ROLL_RATE_DPS     180.0f
#define FC_TARGET_MAX_PITCH_RATE_DPS    180.0f
#define FC_TARGET_MAX_YAW_RATE_DPS      120.0f
#define FC_TARGET_MAX_HOVER_RATE_MPS    1.0f
#define FC_TARGET_HOVER_DELTA_MIN_M     -2.0f
#define FC_TARGET_HOVER_DELTA_MAX_M     2.0f
#define FC_ATTITUDE_KP_DPS_PER_RAD      360.0f
#define FC_TARGET_ATTITUDE_P_DPS_PER_RAD FC_ATTITUDE_KP_DPS_PER_RAD

/* Altitude hold controller. Positive vertical speed is upward. */
#define FC_ESTIMATOR_VERTICAL_SPEED_LPF_ALPHA 0.05f
#define FC_ESTIMATOR_VERTICAL_SPEED_MAX_MPS   5.0f
#define FC_ALTITUDE_HOVER_THROTTLE_NORM       0.50f
#define FC_ALTITUDE_KP_MPS_PER_M              1.00f
#define FC_ALTITUDE_MAX_CLIMB_RATE_MPS        1.00f
#define FC_ALTITUDE_MAX_DESCENT_RATE_MPS      0.70f
#define FC_ALTITUDE_RATE_KP_NORM_PER_MPS      0.20f
#define FC_ALTITUDE_RATE_KI_NORM_PER_MPS      0.08f
#define FC_ALTITUDE_RATE_KD_NORM_PER_MPS      0.00f
#define FC_ALTITUDE_RATE_OUTPUT_LIMIT_NORM    0.30f
#define FC_ALTITUDE_TILT_COMP_MIN_COS         0.70f

/* Commander flight-state machine. */
#define FC_TAKEOFF_TRIGGER_US           (FC_RC_PWM_CENTER_US + FC_RC_HOVER_DEADBAND_US + 50U)
#define FC_TAKEOFF_ALTITUDE_M           0.50f
#define FC_TAKEOFF_CLIMB_RATE_MPS       0.50f
#define FC_TAKEOFF_COMPLETE_MARGIN_M    0.10f
#define FC_LAND_DESCENT_RATE_MPS        0.35f
#define FC_LAND_ALTITUDE_MARGIN_M       0.08f
#define FC_LAND_VERTICAL_SPEED_MAX_MPS  0.15f
#define FC_LAND_SETTLE_MS               1000U
#define FC_ARM_ATTITUDE_MAX_RAD         0.35f

#endif
