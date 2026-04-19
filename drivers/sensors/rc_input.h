#ifndef SENSOR_RC_INPUT_H
#define SENSOR_RC_INPUT_H

#include <stdbool.h>
#include "app/app_types.h"

bool rc_input_init(void);
bool rc_input_poll(rc_input_t *rc);

#endif
