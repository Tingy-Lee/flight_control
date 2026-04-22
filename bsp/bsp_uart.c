#include "bsp/bsp_uart.h"
#include "debug.h"

#define IMU_UART_RX_BUFFER_SIZE  512U
#define IMU_UART_IRQ_PRIORITY    0xF0U
#define UART_TX_TIMEOUT_LOOPS    100000U

static volatile uint8_t g_imu_uart_rx_buffer[IMU_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_imu_uart_rx_write;
static volatile uint16_t g_imu_uart_rx_read;
static volatile uint8_t g_imu_uart_rx_started;

volatile uint32_t g_dbg_usart4_irq_count;
volatile uint32_t g_dbg_usart4_rx_count;
volatile uint32_t g_dbg_usart4_ore_count;
volatile uint32_t g_dbg_usart4_fe_count;
volatile uint32_t g_dbg_usart4_ne_count;
volatile uint32_t g_dbg_usart4_pe_count;
volatile uint32_t g_dbg_usart4_spurious_count;
volatile uint32_t g_dbg_usart4_error_flags;
volatile uint32_t g_dbg_usart4_statr;
volatile uint8_t g_dbg_usart4_last_byte;
volatile uint8_t g_dbg_pf3_level;
volatile uint16_t g_dbg_gpiof_indr;
volatile uint8_t g_dbg_gpiof_pf3_af;
volatile uint32_t g_dbg_afio_pcfr1;
volatile uint32_t g_dbg_usart4_ctlr1;
volatile uint32_t g_dbg_usart4_ctlr2;
volatile uint32_t g_dbg_usart4_ctlr3;
volatile uint32_t g_dbg_usart4_brr;
volatile uint32_t g_dbg_usart4_nvic_enabled;
volatile uint32_t g_dbg_usart4_nvic_pending;
volatile uint32_t g_dbg_usart4_nvic_active;
volatile uint32_t g_dbg_usart4_nvic_own_core;
volatile uint32_t g_dbg_usart4_rx_irq_start_count;
volatile uint32_t g_dbg_uart_tx_timeout_count;

void USART4_IRQHandler(void) __attribute__((interrupt()));

static uint16_t imu_uart_rx_next(uint16_t index)
{
    return (uint16_t)((index + 1U) % IMU_UART_RX_BUFFER_SIZE);
}

static void imu_uart_clear_rx_status(void)
{
    uint16_t guard = 0U;

    while ((USART_GetFlagStatus(USART4, USART_FLAG_RXNE) != RESET) && (guard < IMU_UART_RX_BUFFER_SIZE)) {
        (void)USART_ReceiveData(USART4);
        guard++;
    }

    if ((USART_GetFlagStatus(USART4, USART_FLAG_ORE) != RESET) ||
        (USART_GetFlagStatus(USART4, USART_FLAG_NE) != RESET) ||
        (USART_GetFlagStatus(USART4, USART_FLAG_FE) != RESET) ||
        (USART_GetFlagStatus(USART4, USART_FLAG_PE) != RESET)) {
        (void)USART4->STATR;
        (void)USART_ReceiveData(USART4);
    }
}

static bool uart_wait_txe(USART_TypeDef *usart)
{
    uint32_t timeout = UART_TX_TIMEOUT_LOOPS;

    while (USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET) {
        if (timeout-- == 0U) {
            g_dbg_uart_tx_timeout_count++;
            return false;
        }
    }

    return true;
}

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

void bsp_uart_imu_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};

    g_imu_uart_rx_write = 0U;
    g_imu_uart_rx_read = 0U;
    g_imu_uart_rx_started = 0U;

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART4, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOF | RCC_HB2Periph_AFIO, ENABLE);

    /* USART4: PF4 TX, PF3 RX, AF7. */
    GPIO_SetBits(GPIOF, GPIO_Pin_4);
    gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOF, &gpio);

    GPIO_PinAFConfig(GPIOF, GPIO_PinSource4, GPIO_AF7);
    gpio.GPIO_Pin = GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOF, &gpio);

    GPIO_PinAFConfig(GPIOF, GPIO_PinSource3, GPIO_AF7);
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IPU; /* USART4 RX PF3 idle is high. */
    GPIO_Init(GPIOF, &gpio);

    init_usart_common(USART4, baudrate);
    NVIC_SetAllocateIRQ(USART4_IRQn, Core_ID_V5F);
    NVIC_SetPriority(USART4_IRQn, IMU_UART_IRQ_PRIORITY);
    NVIC_DisableIRQ(USART4_IRQn);
    NVIC_ClearPendingIRQ(USART4_IRQn);
    USART_ITConfig(USART4, USART_IT_RXNE, DISABLE);
    imu_uart_clear_rx_status();
}

void bsp_uart_imu_start_rx_irq(void)
{
    g_dbg_usart4_rx_irq_start_count++;

    if (g_imu_uart_rx_started == 0U) {
        USART_ITConfig(USART4, USART_IT_RXNE, DISABLE);
        imu_uart_clear_rx_status();
        NVIC_ClearPendingIRQ(USART4_IRQn);
        g_imu_uart_rx_started = 1U;
    }

    USART_ITConfig(USART4, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART4_IRQn);
}

void bsp_uart_debug_sample_imu_rx(void)
{
    g_dbg_gpiof_indr = GPIO_ReadInputData(GPIOF);
    g_dbg_pf3_level = (uint8_t)((g_dbg_gpiof_indr & GPIO_Pin_3) != 0U);
    g_dbg_gpiof_pf3_af = (uint8_t)((AFIO->GPIOF_AFLR >> (GPIO_PinSource3 * 4U)) & 0x0FU);
    g_dbg_afio_pcfr1 = AFIO->PCFR1;
    g_dbg_usart4_statr = USART4->STATR;
    g_dbg_usart4_ctlr1 = USART4->CTLR1;
    g_dbg_usart4_ctlr2 = USART4->CTLR2;
    g_dbg_usart4_ctlr3 = USART4->CTLR3;
    g_dbg_usart4_brr = USART4->BRR;
    g_dbg_usart4_nvic_enabled = NVIC_GetStatusIRQ(USART4_IRQn);
    g_dbg_usart4_nvic_pending = NVIC_GetPendingIRQ(USART4_IRQn);
    g_dbg_usart4_nvic_active = NVIC_GetActive(USART4_IRQn);
    g_dbg_usart4_nvic_own_core = NVIC_OwnCoreGetAllocateIRQ(USART4_IRQn);
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

bool bsp_uart_imu_read_byte(uint8_t *byte)
{
    uint32_t irq_enabled;

    if (byte == 0) {
        return false;
    }

    irq_enabled = NVIC_GetStatusIRQ(USART4_IRQn);
    NVIC_DisableIRQ(USART4_IRQn);
    if (g_imu_uart_rx_read == g_imu_uart_rx_write) {
        if (irq_enabled != 0U) {
            NVIC_EnableIRQ(USART4_IRQn);
        }
        return false;
    }

    *byte = g_imu_uart_rx_buffer[g_imu_uart_rx_read];
    g_imu_uart_rx_read = imu_uart_rx_next(g_imu_uart_rx_read);
    if (irq_enabled != 0U) {
        NVIC_EnableIRQ(USART4_IRQn);
    }
    return true;
}

void bsp_uart_gps_write_byte(uint8_t byte)
{
    if (!uart_wait_txe(USART2)) {
        return;
    }

    USART_SendData(USART2, byte);
}

void bsp_uart_imu_write_byte(uint8_t byte)
{
    if (!uart_wait_txe(USART4)) {
        return;
    }

    USART_SendData(USART4, byte);
}

void bsp_uart_imu_write(const uint8_t *data, uint16_t len)
{
    if (data == 0) {
        return;
    }

    for (uint16_t i = 0; i < len; i++) {
        bsp_uart_imu_write_byte(data[i]);
    }
}

void USART4_IRQHandler(void)
{
    const uint16_t status = USART4->STATR;

    g_dbg_usart4_irq_count++;
    g_dbg_usart4_statr = status;
    g_dbg_usart4_error_flags = (uint32_t)(status & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE));

    if ((status & USART_FLAG_ORE) != 0U) {
        g_dbg_usart4_ore_count++;
    }
    if ((status & USART_FLAG_FE) != 0U) {
        g_dbg_usart4_fe_count++;
    }
    if ((status & USART_FLAG_NE) != 0U) {
        g_dbg_usart4_ne_count++;
    }
    if ((status & USART_FLAG_PE) != 0U) {
        g_dbg_usart4_pe_count++;
    }

    if ((status & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)) == 0U) {
        g_dbg_usart4_spurious_count++;
        return;
    }

    g_dbg_usart4_last_byte = (uint8_t)USART4->DATAR;
    if ((status & USART_FLAG_RXNE) != 0U) {
        const uint16_t write = g_imu_uart_rx_write;
        const uint16_t next = (uint16_t)((write + 1U) & (IMU_UART_RX_BUFFER_SIZE - 1U));

        g_dbg_usart4_rx_count++;

        if (next == g_imu_uart_rx_read) {
            g_imu_uart_rx_read = (uint16_t)((g_imu_uart_rx_read + 1U) & (IMU_UART_RX_BUFFER_SIZE - 1U));
        }

        g_imu_uart_rx_buffer[write] = g_dbg_usart4_last_byte;
        g_imu_uart_rx_write = next;
    }
}
