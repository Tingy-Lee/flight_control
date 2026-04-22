#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <stdbool.h>
#include "app/app_types.h"

bool imu_init(void);
bool imu_read(imu_sample_t *sample);
bool imu_read_barometer(baro_sample_t *sample);

#endif
