#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdbool.h>
#include <stdint.h>

void bsp_i2c2_init(uint32_t clock_hz);
bool bsp_i2c2_mem_write(uint8_t dev_addr_7bit, uint8_t reg_addr, const uint8_t *data, uint16_t len);
bool bsp_i2c2_mem_read(uint8_t dev_addr_7bit, uint8_t reg_addr, uint8_t *data, uint16_t len);

#endif
