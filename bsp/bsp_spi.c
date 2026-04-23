#include "bsp/bsp_spi.h"
#include "debug.h"
#include "debug_diagnostics.h"

#define SPI_TIMEOUT_LOOPS 100000U

static bool spi2_transfer_byte(uint8_t tx, uint8_t *rx)
{
    uint32_t timeout = SPI_TIMEOUT_LOOPS;

    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) {
        if (timeout-- == 0U) {
            g_dbg_bus.spi2.timeout_count++;
            return false;
        }
    }

    SPI_I2S_SendData(SPI2, tx);
    timeout = SPI_TIMEOUT_LOOPS;

    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) {
        if (timeout-- == 0U) {
            g_dbg_bus.spi2.timeout_count++;
            return false;
        }
    }

    if (rx != 0) {
        *rx = (uint8_t)SPI_I2S_ReceiveData(SPI2);
    } else {
        (void)SPI_I2S_ReceiveData(SPI2);
    }

    return true;
}

void bsp_spi2_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    SPI_InitTypeDef spi = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_SPI2, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB | RCC_HB2Periph_GPIOC | RCC_HB2Periph_AFIO, ENABLE);

    /* WCH example route: SPI2_SCK PB13, SPI2_MOSI PC1, SPI2_MISO PC2.
     * PB12 is kept as software CS so multiple IMU/baro boards can be tested.
     */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF5);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF5);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF5);

    gpio.GPIO_Pin = GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOB, &gpio);
    bsp_spi2_cs_high();

    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode4;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &spi);
    SPI_Cmd(SPI2, ENABLE);
}

uint8_t bsp_spi2_transfer(uint8_t tx)
{
    uint8_t rx = 0xFFU;

    (void)spi2_transfer_byte(tx, &rx);
    return rx;
}

bool bsp_spi2_transfer_buf(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (len == 0U) {
        return true;
    }

    for (uint16_t i = 0; i < len; i++) {
        const uint8_t out = (tx != 0) ? tx[i] : 0xFFU;
        uint8_t in = 0xFFU;
        if (!spi2_transfer_byte(out, &in)) {
            return false;
        }
        if (rx != 0) {
            rx[i] = in;
        }
    }

    return true;
}

void bsp_spi2_cs_low(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

void bsp_spi2_cs_high(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
}
