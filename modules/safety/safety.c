#include "modules/safety/safety.h"
#include "app/app_config.h"

void safety_init(void)
{
}

void safety_update(flight_state_t *state)
{
    uint32_t errors = 0U;

    if (state == 0) {
        return;
    }

    if (!state->imu.healthy) {
        errors |= FC_ERR_IMU_UNHEALTHY;
    }

    if (!state->baro.healthy) {
        errors |= FC_ERR_BARO_UNHEALTHY;
    }

    if ((!state->rc.healthy) || state->rc.failsafe || state->rc.frame_lost) {
        errors |= FC_ERR_RC_FAILSAFE;
    }

#if !FC_ALLOW_ARMING
    errors |= FC_ERR_ARMING_DISABLED;
#endif

    state->error_flags = errors;
}
