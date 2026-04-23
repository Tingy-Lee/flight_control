/********************************** (C) COPYRIGHT *******************************
 * Flight control entry point for CH32H417 V5F.
 *******************************************************************************/

#include "app/app.h"
#include "bsp/bsp_board.h"
#include "debug.h"
#include "debug_diagnostics.h"
#include <stdint.h>

int main(void)
{
    NVIC_DisableIRQ(USART4_IRQn);
    NVIC_ClearPendingIRQ(USART4_IRQn);

    g_dbg_boot.startup_phase = 1U;
    bsp_board_init();
    g_dbg_boot.startup_phase = 20U;
    app_start();
    g_dbg_boot.startup_phase = 250U;

    while (1) {
    }
}
