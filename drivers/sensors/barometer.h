#ifndef SENSOR_BAROMETER_H
#define SENSOR_BAROMETER_H

#include <stdbool.h>
#include "app/app_types.h"

bool barometer_init(void);
bool barometer_read(baro_sample_t *sample);

#endif
