/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32h417_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_it.h"
#include "debug_diagnostics.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  g_dbg_trap.nmi_count++;
  __disable_irq();
  while (1)
  {
    
  }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  g_dbg_trap.hardfault.count++;
  __asm volatile("csrr %0, mcause" : "=r"(g_dbg_trap.hardfault.mcause));
  __asm volatile("csrr %0, mepc" : "=r"(g_dbg_trap.hardfault.mepc));
  __asm volatile("csrr %0, mtval" : "=r"(g_dbg_trap.hardfault.mtval));
  __asm volatile("csrr %0, mstatus" : "=r"(g_dbg_trap.hardfault.mstatus));
  __asm volatile("mv %0, sp" : "=r"(g_dbg_trap.hardfault.sp));
  __disable_irq();
  while (1)
  {
  }
}


