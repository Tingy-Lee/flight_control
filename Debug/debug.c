/********************************** (C) COPYRIGHT  *******************************
* File Name          : debug.c
* Author             : WCH
* Version            : V1.0.1
* Date               : 2025/07/16
* Description        : This file contains all the functions prototypes for UART
*                      Printf , Delay functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"

static uint16_t  p_us = 0;
static uint32_t p_ms = 0;

#define DEBUG_SYSTICK_ENABLE_BIT  (1U << 0)
#define DEBUG_SYSTICK_DONE_BIT    (1U << 1)
#define DEBUG_DELAY_TIMEOUT_PAD   1024U
#define DEBUG_UART_TIMEOUT_LOOPS  1000000U

volatile uint32_t g_dbg_delay_timeout_count;
volatile uint32_t g_dbg_printf_timeout_count;

static int debug_scheduler_started(void)
{
    return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
}

static void delay_spin(uint32_t cycles)
{
    while (cycles-- != 0U) {
        __asm volatile("nop");
    }
}

static void delay_systick_cycles(uint32_t cycles)
{
    uint32_t timeout;

    if (cycles == 0U) {
        return;
    }

    timeout = cycles + (cycles >> 3) + DEBUG_DELAY_TIMEOUT_PAD;
    SysTick1->ISR &= ~DEBUG_SYSTICK_DONE_BIT;
    SysTick1->CNT = 0;
    SysTick1->CMP = cycles;
    SysTick1->CTLR = (1 << 2);
    SysTick1->CTLR |= DEBUG_SYSTICK_ENABLE_BIT;

    while ((SysTick1->ISR & DEBUG_SYSTICK_DONE_BIT) != DEBUG_SYSTICK_DONE_BIT) {
        if (timeout-- == 0U) {
            g_dbg_delay_timeout_count++;
            break;
        }
    }

    SysTick1->CTLR &= ~DEBUG_SYSTICK_ENABLE_BIT;
}

/*********************************************************************
 * @fn      Delay_Init
 *
 * @brief   Initializes Delay Funcation.
 *
 * @return  none
 */
void Delay_Init(void)
{
    p_us = HCLKClock / 1000000;
    p_ms = (uint16_t)p_us * 1000;
}

/*********************************************************************
 * @fn      Delay_Us
 *
 * @brief   Microsecond Delay Time.
 *
 * @param   n - Microsecond number.
 *
 * @return  none
 */
void Delay_Us(uint32_t n)
{
    const uint32_t cycles = (uint32_t)n * p_us;

    if (debug_scheduler_started()) {
        delay_spin(cycles);
        return;
    }

    delay_systick_cycles(cycles);
}

/*********************************************************************
 * @fn      Delay_Ms
 *
 * @brief   Millisecond Delay Time.
 *
 * @param   n - Millisecond number.
 *
 * @return  none
 */
void Delay_Ms(uint32_t n)
{
    const uint32_t cycles = (uint32_t)n * p_ms;

    if (debug_scheduler_started()) {
        TickType_t ticks = pdMS_TO_TICKS(n);
        if ((ticks == 0U) && (n != 0U)) {
            ticks = 1U;
        }
        vTaskDelay(ticks);
        return;
    }

    delay_systick_cycles(cycles);
}

/*********************************************************************
 * @fn      USART_Printf_Init
 *
 * @brief   Initializes the USARTx peripheral.
 *
 * @param   baudrate - USART communication baud rate.
 *
 * @return  none
 */
void USART_Printf_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

#if(DEBUG == DEBUG_UART1)  
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_USART1 | RCC_HB2Periph_GPIOA, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF7);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

#elif(DEBUG == DEBUG_UART8)
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART8, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF11);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

#elif(DEBUG == DEBUG_UART6)
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART6, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

#endif

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;

#if(DEBUG == DEBUG_UART1)
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

#elif(DEBUG == DEBUG_UART8)
    USART_Init(USART8, &USART_InitStructure);
    USART_Cmd(USART8, ENABLE);
    
#elif(DEBUG == DEBUG_UART6)
    USART_Init(USART6, &USART_InitStructure);
    USART_Cmd(USART6, ENABLE);

#endif
}

/*********************************************************************
 * @fn      _write
 *
 * @brief   Support Printf Function
 *
 * @param   *buf - UART send Data.
 *          size - Data length
 *
 * @return  size: Data length
 */
__attribute__((used)) int _write(int fd, char *buf, int size)
{
    int i = 0;

    for(i = 0; i < size; i++)
    {
        uint32_t timeout = DEBUG_UART_TIMEOUT_LOOPS;
#if(DEBUG == DEBUG_UART1)
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
            if (timeout-- == 0U) {
                g_dbg_printf_timeout_count++;
                return i;
            }
        }
        USART_SendData(USART1, *buf++);
#elif(DEBUG == DEBUG_UART8)
        while(USART_GetFlagStatus(USART8, USART_FLAG_TXE) == RESET) {
            if (timeout-- == 0U) {
                g_dbg_printf_timeout_count++;
                return i;
            }
        }
        USART_SendData(USART8, *buf++);
#elif(DEBUG == DEBUG_UART6)
        while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET) {
            if (timeout-- == 0U) {
                g_dbg_printf_timeout_count++;
                return i;
            }
        }
        USART_SendData(USART6, *buf++);
#endif
    }

    return size;
}

/*********************************************************************
 * @fn      _sbrk
 *
 * @brief   Change the spatial position of data segment.
 *
 * @return  size: Data length
 */
__attribute__((used)) void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end))
    return NULL - 1;

    curbrk += incr;
    return curbrk - incr;
}



