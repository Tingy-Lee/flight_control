#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include "app/app_types.h"

#define FC_ERR_IMU_UNHEALTHY     (1UL << 0)
#define FC_ERR_BARO_UNHEALTHY    (1UL << 1)
#define FC_ERR_RC_FAILSAFE       (1UL << 2)
#define FC_ERR_ARMING_DISABLED   (1UL << 3)

void safety_init(void);
void safety_update(flight_state_t *state);

#endif
