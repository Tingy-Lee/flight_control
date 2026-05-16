#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

void bsp_uart_gps_init(uint32_t baudrate);
void bsp_uart_gps_start_rx_irq(void);
void bsp_uart_rc_init(uint32_t baudrate);
void bsp_uart_rc_start_rx_irq(void);
void bsp_uart_debug_sample_rc_rx(void);
void bsp_uart_imu_init(uint32_t baudrate);
void bsp_uart_imu_start_rx_irq(void);
void bsp_uart_debug_sample_imu_rx(void);
bool bsp_uart_gps_read_byte(uint8_t *byte);
bool bsp_uart_rc_read_byte(uint8_t *byte);
bool bsp_uart_imu_read_byte(uint8_t *byte);
void bsp_uart_gps_write_byte(uint8_t byte);
void bsp_uart_imu_write_byte(uint8_t byte);
void bsp_uart_imu_write(const uint8_t *data, uint16_t len);

#endif
