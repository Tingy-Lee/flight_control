#ifndef LOGGER_H
#define LOGGER_H

#include "app/app_types.h"

void logger_init(void);
void logger_print_state(const flight_state_t *state);

#endif
