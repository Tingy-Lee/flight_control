#ifndef SENSOR_GPS_H
#define SENSOR_GPS_H

#include <stdbool.h>
#include "app/app_types.h"

bool gps_init(void);
bool gps_poll(gps_sample_t *sample);

#endif
