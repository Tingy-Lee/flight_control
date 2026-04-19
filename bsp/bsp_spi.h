#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdbool.h>
#include <stdint.h>

void bsp_spi2_init(void);
uint8_t bsp_spi2_transfer(uint8_t tx);
bool bsp_spi2_transfer_buf(const uint8_t *tx, uint8_t *rx, uint16_t len);
void bsp_spi2_cs_low(void);
void bsp_spi2_cs_high(void);

#endif
