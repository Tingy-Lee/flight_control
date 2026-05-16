#include "modules/estimator/attitude_estimator.h"
#include "app/app_config.h"

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

void attitude_estimator_init(estimator_state_t *state)
{
    if (state == 0) {
        return;
    }

    state->attitude.roll_rad = 0.0f;
    state->attitude.pitch_rad = 0.0f;
    state->attitude.yaw_rad = 0.0f;
    state->altitude_m = 0.0f;
    state->vertical_speed_mps = 0.0f;
    state->attitude_valid = false;
    state->altitude_valid = false;
}

void attitude_estimator_update(estimator_state_t *state, const imu_sample_t *imu, const baro_sample_t *baro, float dt_s)
{
    if ((state == 0) || (imu == 0) || (baro == 0)) {
        return;
    }

    if (imu->healthy) {
        state->attitude = imu->euler_rad;
        state->attitude_valid = true;
    } else {
        state->attitude_valid = false;
    }

    if (baro->healthy) {
        const bool had_altitude = state->altitude_valid;
        const float old_alt = state->altitude_m;
        state->altitude_m = baro->altitude_m;
        if (had_altitude && (dt_s > 0.0f)) {
            const float raw_vz = clamp_float((state->altitude_m - old_alt) / dt_s,
                                             -FC_ESTIMATOR_VERTICAL_SPEED_MAX_MPS,
                                             FC_ESTIMATOR_VERTICAL_SPEED_MAX_MPS);
            state->vertical_speed_mps +=
                FC_ESTIMATOR_VERTICAL_SPEED_LPF_ALPHA * (raw_vz - state->vertical_speed_mps);
        } else {
            state->vertical_speed_mps = 0.0f;
        }
        state->altitude_valid = true;
    } else {
        state->vertical_speed_mps = 0.0f;
        state->altitude_valid = false;
    }
}
