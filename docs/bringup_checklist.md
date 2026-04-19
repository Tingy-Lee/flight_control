# Bring-Up Checklist

## Power-Off Checks

- Confirm no propellers are installed.
- Confirm CH32H417 board and sensor modules share GND.
- Confirm ESC command wires are connected to PE9, PE11, PE13, PE14.
- Confirm ESC power and logic levels are compatible.
- Confirm battery voltage/current ADC dividers do not exceed 3.3 V.

## First Firmware Check

- Build the active V5F image: `make all`.
- Download `obj/FlightControl_V5F.hex`.
- Debug with `FlightControl_V5F.launch` on GDB port `3334`.
- Open serial log at 115200 baud.
- Expected startup log:
  - project name/version
  - V5F core clock
  - motor PWM frequency
  - repeated state log with `armed=0`

## PWM Check

- Measure each motor output with a logic analyzer.
- Expected default:
  - 400 Hz frame
  - 1000 us pulse width
  - motors remain at 1000 us because `FC_ALLOW_ARMING=0`

## Before Any Motor Spin

- Keep `FC_MOTOR_BRINGUP_LIMIT_US` at 1200 us or lower.
- Use bench supply current limit if possible.
- Test one ESC/motor at a time.
- Verify motor order:
  - M1 front-left
  - M2 front-right
  - M3 rear-right
  - M4 rear-left

## Before Propellers

- Implement real IMU health and calibration.
- Implement real RC parser and failsafe.
- Verify disarm switch cuts PWM to 1000 us immediately.
- Verify low-voltage path forces FAILSAFE.
- Verify roll/pitch/yaw stick signs with no propellers.
