# CH32H417 Flight Control

This repository is configured as a single-image V5F flight-control firmware
skeleton for the WCH CH32H417 drone project. The tree is intentionally kept
focused on the active firmware so build, download, and debug stay predictable.

## Current Scope

- Active MRS project: `FlightControl_V5F.wvproj`
- Active firmware image: `obj/FlightControl_V5F.hex`
- Active debug symbol file: `obj/FlightControl_V5F.elf`
- Active core: V5F
- FreeRTOS scheduler at 1 kHz tick
- Safe motor PWM output on TIM1:
  - M1: PE9 / TIM1_CH1
  - M2: PE11 / TIM1_CH2
  - M3: PE13 / TIM1_CH3
  - M4: PE14 / TIM1_CH4
- Default motor output is locked at 1000 us.
- Bring-up motor limit is 1200 us in `app/app_config.h`.
- I2C2 sensor bus: PC0 SCL, PC1 SDA
- GPS UART: USART2, PD5 TX, PD6 RX
- RC UART placeholder: USART3, PB10 TX, PB11 RX
- Battery ADC placeholders:
  - PA0 / ADC0: voltage
  - PA1 / ADC1: current

## Safety Defaults

`FC_ALLOW_ARMING` is currently `0`, so the firmware cannot arm motors even if
the RC parser is later filled in. Change it only after:

1. Four PWM channels are checked with no propellers.
2. Motor order and rotation are verified.
3. RC failsafe behavior is verified.
4. IMU and barometer health checks are real, not stubs.

## Module Layout

- `User/`: startup-facing entry files and FreeRTOS config.
- `tasks/Inc/`: FreeRTOS task declarations.
- `tasks/Src/`: task creation plus sensor, control, commander, and logger loops.
- `bsp/`: CH32H417 board/peripheral setup.
- `drivers/sensors/`: IMU, barometer, GPS, RC input drivers.
- `modules/estimator/`: attitude and altitude estimation.
- `modules/control/`: PID, rate controller, X-quad mixer.
- `modules/navigation/`: future waypoint mission state machine.
- `modules/safety/`: arming/failsafe checks.
- `modules/comm/`: serial logging.

## Build And Debug

1. Build the active V5F firmware from the project root: `make all`.
2. Download `obj/FlightControl_V5F.hex`.
3. Debug with `FlightControl_V5F.launch` or the MRS `FlightControl_V5F` debug configuration.
4. Use GDB port `3334` for the V5F session.
5. Open the serial log at 115200 baud.
6. Confirm startup prints project name/version, core clock, PWM rate, and repeated state logs.
