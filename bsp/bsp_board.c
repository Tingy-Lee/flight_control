#include "bsp/bsp_board.h"
#include "app/app_config.h"
#include "bsp/bsp_adc.h"
#include "bsp/bsp_i2c.h"
#include "bsp/bsp_pwm.h"
#include "bsp/bsp_spi.h"
#include "bsp/bsp_uart.h"
#include "debug.h"
#include "debug_diagnostics.h"

static volatile uint32_t g_board_millis;

void bsp_board_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    g_dbg_boot.startup_phase = 2U;
    SystemAndCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    g_dbg_boot.startup_phase = 3U;

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOB, &gpio);
    bsp_board_led_set(0);

    bsp_pwm_motors_init();

#if FC_ENABLE_I2C2_SENSORS
    bsp_i2c2_init(100000U);
#endif

#if FC_ENABLE_IMU_UART
    bsp_uart_imu_init(FC_IMU_UART_BAUD);
    g_dbg_boot.startup_phase = 10U;
#endif

#if FC_ENABLE_SPI2_IMU
    bsp_spi2_init();
#endif

#if FC_ENABLE_GPS_USART2
    bsp_uart_gps_init(115200U);
#endif

#if FC_ENABLE_RC_USART3
    bsp_uart_rc_init(100000U);
#endif

#if FC_ENABLE_BATTERY_ADC
    bsp_adc_init();
#endif

    printf("\r\n%s %s\r\n", FC_PROJECT_NAME, FC_PROJECT_VERSION);
    printf("CoreClk:%lu Hz, Airframe:%s\r\n", (unsigned long)SystemCoreClock, FC_AIRFRAME_NAME);
    printf("Motors locked at %u us, PWM %u Hz\r\n", FC_MOTOR_PWM_MIN_US, FC_MOTOR_PWM_HZ);
    g_dbg_boot.startup_phase = 19U;
}

void bsp_board_led_set(uint8_t on)
{
    if (on != 0U) {
        GPIO_SetBits(GPIOB, GPIO_Pin_1);
    } else {
        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    }
}

void bsp_board_led_toggle(void)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_1, (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET) ? Bit_SET : Bit_RESET);
}

uint32_t bsp_board_millis(void)
{
    return g_board_millis;
}

void vApplicationTickHook(void)
{
    g_board_millis++;
    g_dbg_boot.tick_hook_count++;
}
