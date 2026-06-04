# 当前接口使用说明

更新日期：2026-05-26

本文档记录当前飞控工程源码中实际启用、初始化或预留的外设接口。结论以 `app/app_config.h` 中的功能开关和 `bsp/` 目录下的外设初始化代码为准。

## 当前启用的接口

| 功能 | 外设 | 引脚 | 配置 | 作用 |
| --- | --- | --- | --- | --- |
| 调试串口 / `printf` | `USART8` | `PB4` = TX，AF11 | 115200，8N1，TX only | 启动日志、调试打印。`DEBUG` 未被编译参数覆盖时默认选择 `DEBUG_UART8`。 |
| IMU / 十轴模块 UART | `USART4` | `PF4` = TX，`PF3` = RX，AF7 | 115200，8N1，RX 中断 | 与十轴 IMU 模块通信。TX 用于发送设置输出频率、算法类型、请求版本等命令；RX 接收 IMU 自动上报的加速度、陀螺仪、欧拉角和气压高度数据。 |
| GPS | `USART2` | `PD5` = TX，`PD6` = RX，AF7 | 9600，8N1，RX 中断 | 接收 GPS NMEA 语句并解析定位、速度、卫星数等信息。TX 已配置，但当前 GPS 驱动主要使用 RX。 |
| 遥控器输入 | `USART3` | `PB10` = TX，`PB11` = RX，AF7 | 115200，8N1，RX 中断 | 接收 FlySky iBUS 遥控器数据，解析通道值、解锁开关和 failsafe 状态。TX 已配置，但当前遥控器驱动主要使用 RX。 |
| 电机 PWM | `TIM1` | M1: `PA8` = `TIM1_CH1`，M2: `PE11` = `TIM1_CH2`，M3: `PE13` = `TIM1_CH3`，M4: `PE14` = `TIM1_CH4`，AF1 | 50 Hz，1000-2000 us | 输出四路电调油门脉宽。启动后默认锁定在 1000 us，当前无桨测试限幅为 2000 us。 |
| 电池采样 | `ADC1` | `PA0` = `ADC_Channel_0`，`PA1` = `ADC_Channel_1` | 单次转换，3.3 V 参考，12 bit | 预留为电池电压和电流采样。当前会初始化 ADC，但业务逻辑里尚未周期读取到飞控状态。 |
| 状态 LED | GPIO | `PB1` | 推挽输出 | `CommanderTask` 中周期翻转，用作任务运行/状态心跳指示。 |

## 遥控器协议说明

当前工程使用的是 FlySky iBUS，不是 SBUS。

| 项目 | 当前配置 |
| --- | --- |
| 物理接口 | `USART3` |
| 引脚 | `PB10` TX，`PB11` RX |
| 波特率 | 115200 |
| 串口格式 | 8N1，普通空闲高电平 UART |
| 帧格式 | iBUS 32 字节帧，帧头 `0x20 0x40`，末尾 2 字节 checksum |
| 通道数 | 协议最多解析 14 路，飞控当前使用前 10 路 |
| 当前通道映射 | CH1 roll，CH2 pitch，CH3 throttle，CH4 yaw，CH5 hover-height，CH7 arm；CH7 默认高电平/约 2000 us 为解锁 |

如果后续要接 SBUS，需要新增或修改遥控器驱动。SBUS 通常不是当前 iBUS 的 115200/8N1/非反相配置；当前源码里没有 SBUS 解析、反相输入或 SBUS 串口格式配置。

## 当前未启用但代码已保留的接口

| 功能开关 | 外设 | 引脚 | 当前状态 | 说明 |
| --- | --- | --- | --- | --- |
| `FC_ENABLE_I2C2_SENSORS = 0` | `I2C2` | `PC0` = SCL，`PC1` = SDA，AF9 | 关闭 | 原计划作为传感器总线，100 kHz。IMU/baro 的 I2C 备用路径使用 7-bit 地址 `0x23`。 |
| `FC_ENABLE_SPI2_IMU = 0` | `SPI2` | `PB13` = SCK，`PC1` = MOSI，`PC2` = MISO，`PB12` = 软件 CS，AF5 | 关闭 | 预留给 SPI IMU。注意 `PC1` 与 I2C2 SDA 冲突，所以当前配置中 SPI2 和 I2C2 不应同时启用。 |
| `FC_ENABLE_RUNTIME_LOGGER = 0` | 调试串口 | 跟随 `DEBUG` 串口 | 关闭 | 运行期状态日志关闭；启动时仍会通过调试串口打印项目名、时钟和 PWM 信息。 |

## USART 对照

| USART | 当前用途 | 当前引脚 | 备注 |
| --- | --- | --- | --- |
| `USART1` | 当前未启用 | 代码备用配置只有 `PA9` TX，AF7 | 只有选择 `DEBUG_UART1` 时才会作为 `printf` TX 使用；未配置 RX。 |
| `USART2` | GPS | `PD5` TX，`PD6` RX，AF7 | 当前启用，9600 baud。 |
| `USART3` | 遥控器 iBUS | `PB10` TX，`PB11` RX，AF7 | 当前启用，115200 baud。不是 SBUS。 |
| `USART4` | IMU / 气压高度数据 | `PF4` TX，`PF3` RX，AF7 | 当前启用，115200 baud。 |
| `USART6` | 当前未启用 | 代码备用配置只有 `PB9` TX，AF8 | 只有选择 `DEBUG_UART6` 时才会作为 `printf` TX 使用；未配置 RX。 |
| `USART8` | 调试串口 / `printf` | `PB4` TX，AF11 | 当前默认启用，115200 baud，TX only。 |

## 源码位置

| 内容 | 源码 |
| --- | --- |
| 功能开关、波特率、PWM 参数、遥控通道映射 | `app/app_config.h` |
| 板级初始化顺序 | `bsp/bsp_board.c` |
| 调试串口 `USART8/USART1/USART6` 选择 | `Debug/debug.h`，`Debug/debug.c` |
| GPS / RC / IMU UART 引脚与中断 | `bsp/bsp_uart.c` |
| 电机 PWM 引脚和定时器 | `bsp/bsp_pwm.c` |
| ADC 电池采样引脚 | `bsp/bsp_adc.c` |
| I2C2 预留传感器总线 | `bsp/bsp_i2c.c` |
| SPI2 预留 IMU 总线 | `bsp/bsp_spi.c` |
| iBUS 遥控器解析 | `drivers/sensors/rc_input.c` |
| IMU UART 协议解析 | `drivers/sensors/imu.c` |
