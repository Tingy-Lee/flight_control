#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "app/app_types.h"

void flight_controller_init(void);
void flight_controller_update(flight_state_t *state, float dt_s);

#endif
