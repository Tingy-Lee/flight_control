#include "drivers/sensors/rc_input.h"
#include "app/app_config.h"
#include "bsp/bsp_board.h"
#include "bsp/bsp_uart.h"

#define RC_INPUT_POLL_MAX_BYTES 128U

bool rc_input_init(void)
{
    return true;
}

bool rc_input_poll(rc_input_t *rc)
{
    uint8_t byte;
    uint16_t budget = RC_INPUT_POLL_MAX_BYTES;

    while ((budget-- != 0U) && bsp_uart_rc_read_byte(&byte)) {
        /* TODO: add SBUS/CRSF parser. */
        (void)byte;
    }

    if (rc == 0) {
        return false;
    }

    rc->timestamp_ms = bsp_board_millis();
    rc->roll_us = 1500U;
    rc->pitch_us = 1500U;
    rc->throttle_us = FC_MOTOR_PWM_MIN_US;
    rc->yaw_us = 1500U;
    rc->arm_switch = false;
    rc->failsafe = true;
    rc->healthy = false;

    return true;
}
