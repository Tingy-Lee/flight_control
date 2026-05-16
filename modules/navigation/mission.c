#include "modules/navigation/mission.h"

void mission_init(void)
{
}

void mission_update(flight_state_t *state)
{
    if (state == 0) {
        return;
    }

    /* TODO: add GPS waypoint state machine after stable manual/alt-hold flight.
     * Mode changes are owned by CommanderTask.
     */
}
