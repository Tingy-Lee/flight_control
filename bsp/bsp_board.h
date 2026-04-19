#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include <stdint.h>

void bsp_board_init(void);
void bsp_board_led_set(uint8_t on);
void bsp_board_led_toggle(void);
uint32_t bsp_board_millis(void);

#endif
