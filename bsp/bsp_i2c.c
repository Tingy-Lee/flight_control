#include "bsp/bsp_i2c.h"
#include "debug.h"

#define I2C_TIMEOUT_LOOPS  100000U

static bool wait_event(uint32_t event)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;

    while (!I2C_CheckEvent(I2C2, event)) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}

static bool wait_not_busy(void)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;

    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) != RESET) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}

void bsp_i2c2_init(uint32_t clock_hz)
{
    GPIO_InitTypeDef gpio = {0};
    I2C_InitTypeDef i2c = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_I2C2, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOC | RCC_HB2Periph_AFIO, ENABLE);

    /* WCH example route: I2C2_SCL PC0, I2C2_SDA PC1, AF9. */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource0, GPIO_AF9);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF9);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOC, &gpio);

    i2c.I2C_ClockSpeed = clock_hz;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_16_9;
    i2c.I2C_OwnAddress1 = 0x30;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2, &i2c);
    I2C_Cmd(I2C2, ENABLE);
}

bool bsp_i2c2_mem_write(uint8_t dev_addr_7bit, uint8_t reg_addr, const uint8_t *data, uint16_t len)
{
    if (!wait_not_busy()) {
        return false;
    }

    I2C_GenerateSTART(I2C2, ENABLE);
    if (!wait_event(I2C_EVENT_MASTER_MODE_SELECT)) {
        return false;
    }

    I2C_Send7bitAddress(I2C2, (uint8_t)(dev_addr_7bit << 1), I2C_Direction_Transmitter);
    if (!wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        return false;
    }

    I2C_SendData(I2C2, reg_addr);
    if (!wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        return false;
    }

    for (uint16_t i = 0; i < len; i++) {
        I2C_SendData(I2C2, data[i]);
        if (!wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
            return false;
        }
    }

    I2C_GenerateSTOP(I2C2, ENABLE);
    return true;
}

bool bsp_i2c2_mem_read(uint8_t dev_addr_7bit, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0U)) {
        return false;
    }

    if (!bsp_i2c2_mem_write(dev_addr_7bit, reg_addr, 0, 0)) {
        return false;
    }

    I2C_GenerateSTART(I2C2, ENABLE);
    if (!wait_event(I2C_EVENT_MASTER_MODE_SELECT)) {
        return false;
    }

    I2C_Send7bitAddress(I2C2, (uint8_t)(dev_addr_7bit << 1), I2C_Direction_Receiver);
    if (!wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        return false;
    }

    for (uint16_t i = 0; i < len; i++) {
        if (i == (uint16_t)(len - 1U)) {
            I2C_AcknowledgeConfig(I2C2, DISABLE);
            I2C_GenerateSTOP(I2C2, ENABLE);
        }

        if (!wait_event(I2C_EVENT_MASTER_BYTE_RECEIVED)) {
            I2C_AcknowledgeConfig(I2C2, ENABLE);
            return false;
        }

        data[i] = I2C_ReceiveData(I2C2);
    }

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return true;
}
