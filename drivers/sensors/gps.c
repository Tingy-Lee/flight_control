#include "drivers/sensors/gps.h"
#include "bsp/bsp_board.h"
#include "bsp/bsp_uart.h"

bool gps_init(void)
{
    return true;
}

bool gps_poll(gps_sample_t *sample)
{
    uint8_t byte;

    while (bsp_uart_gps_read_byte(&byte)) {
        /* TODO: feed a tiny NMEA parser. For now we only drain the UART FIFO. */
        (void)byte;
    }

    if (sample == 0) {
        return false;
    }

    sample->timestamp_ms = bsp_board_millis();
    sample->latitude_deg = 0.0;
    sample->longitude_deg = 0.0;
    sample->altitude_m = 0.0f;
    sample->ground_speed_mps = 0.0f;
    sample->satellites = 0U;
    sample->fix_valid = false;

    return true;
}
