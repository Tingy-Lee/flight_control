#include "modules/estimator/attitude_estimator.h"

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
        const float old_alt = state->altitude_m;
        state->altitude_m = baro->altitude_m;
        state->vertical_speed_mps = (dt_s > 0.0f) ? ((state->altitude_m - old_alt) / dt_s) : 0.0f;
        state->altitude_valid = true;
    } else {
        state->altitude_valid = false;
    }
}
