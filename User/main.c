/********************************** (C) COPYRIGHT *******************************
 * Flight control entry point for CH32H417 V5F.
 *******************************************************************************/

#include "app/app.h"
#include "bsp/bsp_board.h"

int main(void)
{
    bsp_board_init();
    app_start();

    while (1) {
    }
}
