#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "app/app_config.h"

typedef struct {
    float x;
    float y;
    float z;
} vec3f_t;

typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} euler_t;

typedef struct {
    uint32_t timestamp_ms;
    euler_t euler_rad;
    vec3f_t gyro_dps;
    vec3f_t accel_g;
    float temperature_c;
    bool healthy;
} imu_sample_t;

typedef struct {
    uint32_t timestamp_ms;
    float pressure_pa;
    float pressure_reference_pa;
    float temperature_c;
    float altitude_m;
    bool healthy;
} baro_sample_t;

typedef struct {
    uint32_t timestamp_ms;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float ground_speed_mps;
    uint8_t satellites;
    bool fix_valid;
} gps_sample_t;

typedef struct {
    uint32_t timestamp_ms;
    uint16_t channels_us[FC_RC_INPUT_CHANNEL_COUNT];
    uint16_t roll_us;
    uint16_t pitch_us;
    uint16_t throttle_us;
    uint16_t yaw_us;
    bool arm_switch;
    bool frame_lost;
    bool failsafe;
    bool healthy;
} rc_input_t;

typedef struct {
    uint32_t timestamp_ms;
    float roll_rad;
    float pitch_rad;
    float yaw_rate_dps;
    float hover_height_delta_m;
    float hover_height_rate_mps;
    bool valid;
} target_state_t;

typedef struct {
    euler_t attitude;
    float altitude_m;
    float vertical_speed_mps;
    bool attitude_valid;
    bool altitude_valid;
} estimator_state_t;

typedef enum {
    FLIGHT_MODE_LOCKED = 0,
    FLIGHT_MODE_STANDBY,
    FLIGHT_MODE_ARMED_IDLE,
    FLIGHT_MODE_TAKEOFF,
    FLIGHT_MODE_ALT_HOLD,
    FLIGHT_MODE_MANUAL_STAB,
    FLIGHT_MODE_MISSION,
    FLIGHT_MODE_LAND,
    FLIGHT_MODE_FAILSAFE
} flight_mode_t;

typedef struct {
    /* Desired body rates from the attitude/yaw command layer. */
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;
    /* Mixer control efforts from the inner rate loops. */
    float roll_cmd;
    float pitch_cmd;
    float yaw_cmd;
    float altitude_target_m;
    float vertical_speed_target_mps;
    float throttle_norm;
} control_setpoint_t;

typedef struct {
    uint16_t motor_us[FC_MOTOR_COUNT];
} motor_outputs_t;

typedef struct {
    imu_sample_t imu;
    baro_sample_t baro;
    gps_sample_t gps;
    rc_input_t rc;
    target_state_t target;
    estimator_state_t estimate;
    control_setpoint_t setpoint;
    motor_outputs_t motors;
    flight_mode_t mode;
    uint32_t mode_entry_ms;
    float home_altitude_m;
    float mode_start_altitude_m;
    float mode_target_altitude_m;
    bool armed;
    bool home_altitude_valid;
    bool mode_altitude_valid;
    uint32_t error_flags;
} flight_state_t;

#endif
