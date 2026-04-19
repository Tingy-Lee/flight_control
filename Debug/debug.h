/********************************** (C) COPYRIGHT  *******************************
* File Name          : debug.h
* Author             : WCH
* Version            : V1.0.1
* Date               : 2025/09/16
* Description        : This file contains all the functions prototypes for UART
*                      Printf , Delay functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdio.h"
#include "ch32h417.h"

/* UART Printf Definition */
#define DEBUG_UART1    1
#define DEBUG_UART8    2
#define DEBUG_UART6    3

/* DEBUG UART Definition */
#ifndef DEBUG
#define DEBUG   DEBUG_UART8
#endif

#ifndef Core_V5F_StartAddr
#define Core_V5F_StartAddr   0x00010000
#endif

extern volatile uint32_t WFE_WkupSource;

void Delay_Init(void);
void Delay_Us (uint32_t n);
void Delay_Ms (uint32_t n);
void USART_Printf_Init(uint32_t baudrate);

#ifdef __cplusplus
}
#endif

#endif 



