#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include "app/app_types.h"

void attitude_estimator_init(estimator_state_t *state);
void attitude_estimator_update(estimator_state_t *state, const imu_sample_t *imu, const baro_sample_t *baro, float dt_s);

#endif
