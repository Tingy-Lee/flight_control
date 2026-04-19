#include "bsp/bsp_uart.h"
#include "debug.h"

static void init_usart_common(USART_TypeDef *usart, uint32_t baudrate)
{
    USART_InitTypeDef cfg = {0};

    cfg.USART_BaudRate = baudrate;
    cfg.USART_WordLength = USART_WordLength_8b;
    cfg.USART_StopBits = USART_StopBits_1;
    cfg.USART_Parity = USART_Parity_No;
    cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    cfg.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(usart, &cfg);
    USART_Cmd(usart, ENABLE);
}

void bsp_uart_gps_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART2, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOD | RCC_HB2Periph_AFIO, ENABLE);

    /* USART2: PD5 TX, PD6 RX, AF7. */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF7);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF7);

    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);

    init_usart_common(USART2, baudrate);
}

void bsp_uart_rc_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART3, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB | RCC_HB2Periph_AFIO, ENABLE);

    /* USART3: PB10 TX, PB11 RX, AF7. */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF7);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF7);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

    init_usart_common(USART3, baudrate);
}

bool bsp_uart_gps_read_byte(uint8_t *byte)
{
    if ((byte != 0) && (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)) {
        *byte = (uint8_t)USART_ReceiveData(USART2);
        return true;
    }

    return false;
}

bool bsp_uart_rc_read_byte(uint8_t *byte)
{
    if ((byte != 0) && (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET)) {
        *byte = (uint8_t)USART_ReceiveData(USART3);
        return true;
    }

    return false;
}

void bsp_uart_gps_write_byte(uint8_t byte)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
    }

    USART_SendData(USART2, byte);
}
