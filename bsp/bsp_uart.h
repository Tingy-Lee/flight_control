#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

void bsp_uart_gps_init(uint32_t baudrate);
void bsp_uart_rc_init(uint32_t baudrate);
bool bsp_uart_gps_read_byte(uint8_t *byte);
bool bsp_uart_rc_read_byte(uint8_t *byte);
void bsp_uart_gps_write_byte(uint8_t byte);

#endif
