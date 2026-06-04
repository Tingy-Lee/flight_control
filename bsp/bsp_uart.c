#include "bsp/bsp_uart.h"
#include "debug.h"
#include "debug_diagnostics.h"

#define GPS_UART_RX_BUFFER_SIZE  1024U
#define GPS_UART_IRQ_PRIORITY    0xF0U
#define RC_UART_RX_BUFFER_SIZE   256U
#define RC_UART_IRQ_PRIORITY     0xF0U
#define IMU_UART_RX_BUFFER_SIZE  512U
#define IMU_UART_IRQ_PRIORITY    0xF0U
#define UART_TX_TIMEOUT_LOOPS    100000U

static volatile uint8_t g_gps_uart_rx_buffer[GPS_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_gps_uart_rx_write;
static volatile uint16_t g_gps_uart_rx_read;
static volatile uint8_t g_gps_uart_rx_started;

static volatile uint8_t g_rc_uart_rx_buffer[RC_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_rc_uart_rx_write;
static volatile uint16_t g_rc_uart_rx_read;
static volatile uint8_t g_rc_uart_rx_started;

static volatile uint8_t g_imu_uart_rx_buffer[IMU_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_imu_uart_rx_write;
static volatile uint16_t g_imu_uart_rx_read;
static volatile uint8_t g_imu_uart_rx_started;

void USART2_IRQHandler(void) __attribute__((interrupt()));
void USART3_IRQHandler(void) __attribute__((interrupt()));
void USART4_IRQHandler(void) __attribute__((interrupt()));

static uint16_t gps_uart_rx_next(uint16_t index)
{
    return (uint16_t)((index + 1U) % GPS_UART_RX_BUFFER_SIZE);
}

static void gps_uart_clear_rx_status(void)
{
    uint16_t guard = 0U;

    while ((USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) && (guard < GPS_UART_RX_BUFFER_SIZE)) {
        (void)USART_ReceiveData(USART2);
        guard++;
    }

    if ((USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET) ||
        (USART_GetFlagStatus(USART2, USART_FLAG_NE) != RESET) ||
        (USART_GetFlagStatus(USART2, USART_FLAG_FE) != RESET) ||
        (USART_GetFlagStatus(USART2, USART_FLAG_PE) != RESET)) {
        (void)USART2->STATR;
        (void)USART_ReceiveData(USART2);
    }
}

static uint16_t rc_uart_rx_next(uint16_t index)
{
    return (uint16_t)((index + 1U) % RC_UART_RX_BUFFER_SIZE);
}

static void rc_uart_clear_rx_status(void)
{
    uint16_t guard = 0U;

    while ((USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET) && (guard < RC_UART_RX_BUFFER_SIZE)) {
        (void)USART_ReceiveData(USART3);
        guard++;
    }

    if ((USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET) ||
        (USART_GetFlagStatus(USART3, USART_FLAG_NE) != RESET) ||
        (USART_GetFlagStatus(USART3, USART_FLAG_FE) != RESET) ||
        (USART_GetFlagStatus(USART3, USART_FLAG_PE) != RESET)) {
        (void)USART3->STATR;
        (void)USART_ReceiveData(USART3);
    }
}

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
            g_dbg_uart.tx_timeout_count++;
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

    g_gps_uart_rx_write = 0U;
    g_gps_uart_rx_read = 0U;
    g_gps_uart_rx_started = 0U;

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
    gpio.GPIO_Mode = GPIO_Mode_IPU; /* GPS NMEA UART idles high. */
    GPIO_Init(GPIOD, &gpio);

    init_usart_common(USART2, baudrate);
    NVIC_SetAllocateIRQ(USART2_IRQn, Core_ID_V5F);
    NVIC_SetPriority(USART2_IRQn, GPS_UART_IRQ_PRIORITY);
    NVIC_DisableIRQ(USART2_IRQn);
    NVIC_ClearPendingIRQ(USART2_IRQn);
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    gps_uart_clear_rx_status();
}

void bsp_uart_gps_start_rx_irq(void)
{
    g_dbg_uart.gps.rx_irq_start_count++;

    if (g_gps_uart_rx_started == 0U) {
        USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
        gps_uart_clear_rx_status();
        NVIC_ClearPendingIRQ(USART2_IRQn);
        g_gps_uart_rx_started = 1U;
    }

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART2_IRQn);
}

void bsp_uart_rc_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};

    g_rc_uart_rx_write = 0U;
    g_rc_uart_rx_read = 0U;
    g_rc_uart_rx_started = 0U;

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
    gpio.GPIO_Mode = GPIO_Mode_IPU; /* iBUS is a normal idle-high UART signal. */
    GPIO_Init(GPIOB, &gpio);

    init_usart_common(USART3, baudrate);
    NVIC_SetAllocateIRQ(USART3_IRQn, Core_ID_V5F);
    NVIC_SetPriority(USART3_IRQn, RC_UART_IRQ_PRIORITY);
    NVIC_DisableIRQ(USART3_IRQn);
    NVIC_ClearPendingIRQ(USART3_IRQn);
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
    rc_uart_clear_rx_status();
}

void bsp_uart_rc_start_rx_irq(void)
{
    g_dbg_uart.rc.rx_irq_start_count++;

    if (g_rc_uart_rx_started == 0U) {
        USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
        rc_uart_clear_rx_status();
        NVIC_ClearPendingIRQ(USART3_IRQn);
        g_rc_uart_rx_started = 1U;
    }

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART3_IRQn);
}

void bsp_uart_debug_sample_rc_rx(void)
{
    g_dbg_uart.rc.rx_pin.gpiob_indr = GPIO_ReadInputData(GPIOB);
    g_dbg_uart.rc.rx_pin.pb11_level = (uint8_t)((g_dbg_uart.rc.rx_pin.gpiob_indr & GPIO_Pin_11) != 0U);
    g_dbg_uart.rc.rx_pin.pb11_af = (uint8_t)((AFIO->GPIOB_AFHR >> ((GPIO_PinSource11 - 8U) * 4U)) & 0x0FU);
    g_dbg_uart.rc.rx_pin.afio_pcfr1 = AFIO->PCFR1;
    g_dbg_uart.rc.regs.statr = USART3->STATR;
    g_dbg_uart.rc.regs.ctlr1 = USART3->CTLR1;
    g_dbg_uart.rc.regs.ctlr2 = USART3->CTLR2;
    g_dbg_uart.rc.regs.ctlr3 = USART3->CTLR3;
    g_dbg_uart.rc.regs.brr = USART3->BRR;
    g_dbg_uart.rc.nvic.enabled = NVIC_GetStatusIRQ(USART3_IRQn);
    g_dbg_uart.rc.nvic.pending = NVIC_GetPendingIRQ(USART3_IRQn);
    g_dbg_uart.rc.nvic.active = NVIC_GetActive(USART3_IRQn);
    g_dbg_uart.rc.nvic.own_core = NVIC_OwnCoreGetAllocateIRQ(USART3_IRQn);
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
    g_dbg_uart.imu.rx_irq_start_count++;

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
    g_dbg_uart.imu.rx_pin.gpiof_indr = GPIO_ReadInputData(GPIOF);
    g_dbg_uart.imu.rx_pin.pf3_level = (uint8_t)((g_dbg_uart.imu.rx_pin.gpiof_indr & GPIO_Pin_3) != 0U);
    g_dbg_uart.imu.rx_pin.pf3_af = (uint8_t)((AFIO->GPIOF_AFLR >> (GPIO_PinSource3 * 4U)) & 0x0FU);
    g_dbg_uart.imu.rx_pin.afio_pcfr1 = AFIO->PCFR1;
    g_dbg_uart.imu.regs.statr = USART4->STATR;
    g_dbg_uart.imu.regs.ctlr1 = USART4->CTLR1;
    g_dbg_uart.imu.regs.ctlr2 = USART4->CTLR2;
    g_dbg_uart.imu.regs.ctlr3 = USART4->CTLR3;
    g_dbg_uart.imu.regs.brr = USART4->BRR;
    g_dbg_uart.imu.nvic.enabled = NVIC_GetStatusIRQ(USART4_IRQn);
    g_dbg_uart.imu.nvic.pending = NVIC_GetPendingIRQ(USART4_IRQn);
    g_dbg_uart.imu.nvic.active = NVIC_GetActive(USART4_IRQn);
    g_dbg_uart.imu.nvic.own_core = NVIC_OwnCoreGetAllocateIRQ(USART4_IRQn);
}

bool bsp_uart_gps_read_byte(uint8_t *byte)
{
    uint32_t irq_enabled;

    if (byte == 0) {
        return false;
    }

    irq_enabled = NVIC_GetStatusIRQ(USART2_IRQn);
    NVIC_DisableIRQ(USART2_IRQn);
    if (g_gps_uart_rx_read == g_gps_uart_rx_write) {
        g_dbg_uart.gps.empty_read_count++;
        g_dbg_uart.gps.rx_read = g_gps_uart_rx_read;
        g_dbg_uart.gps.rx_write = g_gps_uart_rx_write;
        if (irq_enabled != 0U) {
            NVIC_EnableIRQ(USART2_IRQn);
        }
        return false;
    }

    *byte = g_gps_uart_rx_buffer[g_gps_uart_rx_read];
    g_gps_uart_rx_read = gps_uart_rx_next(g_gps_uart_rx_read);
    g_dbg_uart.gps.read_count++;
    g_dbg_uart.gps.rx_read = g_gps_uart_rx_read;
    g_dbg_uart.gps.rx_write = g_gps_uart_rx_write;
    if (irq_enabled != 0U) {
        NVIC_EnableIRQ(USART2_IRQn);
    }
    return true;
}

bool bsp_uart_rc_read_byte(uint8_t *byte)
{
    uint32_t irq_enabled;

    if (byte == 0) {
        return false;
    }

    irq_enabled = NVIC_GetStatusIRQ(USART3_IRQn);
    NVIC_DisableIRQ(USART3_IRQn);
    if (g_rc_uart_rx_read == g_rc_uart_rx_write) {
        if (irq_enabled != 0U) {
            NVIC_EnableIRQ(USART3_IRQn);
        }
        return false;
    }

    *byte = g_rc_uart_rx_buffer[g_rc_uart_rx_read];
    g_rc_uart_rx_read = rc_uart_rx_next(g_rc_uart_rx_read);
    if (irq_enabled != 0U) {
        NVIC_EnableIRQ(USART3_IRQn);
    }
    return true;
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
    g_dbg_uart.imu.read_count++;
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

void USART2_IRQHandler(void)
{
    const uint16_t status = USART2->STATR;

    g_dbg_uart.gps.irq_count++;
    g_dbg_uart.gps.regs.statr = status;
    g_dbg_uart.gps.error_flags = (uint32_t)(status & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE));

    if ((status & USART_FLAG_ORE) != 0U) {
        g_dbg_uart.gps.ore_count++;
    }
    if ((status & USART_FLAG_FE) != 0U) {
        g_dbg_uart.gps.fe_count++;
    }
    if ((status & USART_FLAG_NE) != 0U) {
        g_dbg_uart.gps.ne_count++;
    }
    if ((status & USART_FLAG_PE) != 0U) {
        g_dbg_uart.gps.pe_count++;
    }

    if ((status & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)) == 0U) {
        g_dbg_uart.gps.spurious_count++;
        return;
    }

    if ((status & USART_FLAG_RXNE) != 0U) {
        const uint8_t byte = (uint8_t)USART2->DATAR;
        const uint16_t write = g_gps_uart_rx_write;
        const uint16_t next = gps_uart_rx_next(write);

        if (next == g_gps_uart_rx_read) {
            g_dbg_uart.gps.overflow_count++;
            g_gps_uart_rx_read = gps_uart_rx_next(g_gps_uart_rx_read);
        }

        g_dbg_uart.gps.rx_count++;
        g_dbg_uart.gps.last_byte = byte;
        g_gps_uart_rx_buffer[write] = byte;
        g_gps_uart_rx_write = next;
        g_dbg_uart.gps.rx_read = g_gps_uart_rx_read;
        g_dbg_uart.gps.rx_write = g_gps_uart_rx_write;
        return;
    }

    (void)USART2->DATAR;
}

void USART3_IRQHandler(void)
{
    const uint16_t status = USART3->STATR;

    g_dbg_uart.rc.irq_count++;
    g_dbg_uart.rc.regs.statr = status;
    g_dbg_uart.rc.error_flags = (uint32_t)(status & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE));

    if ((status & USART_FLAG_ORE) != 0U) {
        g_dbg_uart.rc.ore_count++;
    }
    if ((status & USART_FLAG_FE) != 0U) {
        g_dbg_uart.rc.fe_count++;
    }
    if ((status & USART_FLAG_NE) != 0U) {
        g_dbg_uart.rc.ne_count++;
    }
    if ((status & USART_FLAG_PE) != 0U) {
        g_dbg_uart.rc.pe_count++;
    }

    if ((status & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)) == 0U) {
        g_dbg_uart.rc.spurious_count++;
        return;
    }

    if ((status & USART_FLAG_RXNE) != 0U) {
        const uint8_t byte = (uint8_t)USART3->DATAR;
        const uint16_t write = g_rc_uart_rx_write;
        const uint16_t next = (uint16_t)((write + 1U) & (RC_UART_RX_BUFFER_SIZE - 1U));

        g_dbg_uart.rc.rx_count++;
        g_dbg_uart.rc.last_byte = byte;

        if (next == g_rc_uart_rx_read) {
            g_dbg_uart.rc.overflow_count++;
            g_rc_uart_rx_read = (uint16_t)((g_rc_uart_rx_read + 1U) & (RC_UART_RX_BUFFER_SIZE - 1U));
        }

        g_rc_uart_rx_buffer[write] = byte;
        g_rc_uart_rx_write = next;
        return;
    }

    (void)USART3->DATAR;
}

void USART4_IRQHandler(void)
{
    const uint16_t status = USART4->STATR;

    g_dbg_uart.imu.irq_count++;
    g_dbg_uart.imu.regs.statr = status;
    g_dbg_uart.imu.error_flags = (uint32_t)(status & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE));

    if ((status & USART_FLAG_ORE) != 0U) {
        g_dbg_uart.imu.ore_count++;
    }
    if ((status & USART_FLAG_FE) != 0U) {
        g_dbg_uart.imu.fe_count++;
    }
    if ((status & USART_FLAG_NE) != 0U) {
        g_dbg_uart.imu.ne_count++;
    }
    if ((status & USART_FLAG_PE) != 0U) {
        g_dbg_uart.imu.pe_count++;
    }

    if ((status & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)) == 0U) {
        g_dbg_uart.imu.spurious_count++;
        return;
    }

    g_dbg_uart.imu.last_byte = (uint8_t)USART4->DATAR;
    if ((status & USART_FLAG_RXNE) != 0U) {
        const uint16_t write = g_imu_uart_rx_write;
        const uint16_t next = (uint16_t)((write + 1U) & (IMU_UART_RX_BUFFER_SIZE - 1U));

        g_dbg_uart.imu.rx_count++;

        if (next == g_imu_uart_rx_read) {
            g_imu_uart_rx_read = (uint16_t)((g_imu_uart_rx_read + 1U) & (IMU_UART_RX_BUFFER_SIZE - 1U));
        }

        g_imu_uart_rx_buffer[write] = g_dbg_uart.imu.last_byte;
        g_imu_uart_rx_write = next;
    }
}
