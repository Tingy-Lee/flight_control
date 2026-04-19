################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/bsp_adc.c \
../bsp/bsp_board.c \
../bsp/bsp_i2c.c \
../bsp/bsp_pwm.c \
../bsp/bsp_spi.c \
../bsp/bsp_uart.c 

C_DEPS += \
./bsp/bsp_adc.d \
./bsp/bsp_board.d \
./bsp/bsp_i2c.d \
./bsp/bsp_pwm.d \
./bsp/bsp_spi.d \
./bsp/bsp_uart.d 

OBJS += \
./bsp/bsp_adc.o \
./bsp/bsp_board.o \
./bsp/bsp_i2c.o \
./bsp/bsp_pwm.o \
./bsp/bsp_spi.o \
./bsp/bsp_uart.o 

DIR_OBJS += \
./bsp/*.o \

DIR_DEPS += \
./bsp/*.d \

DIR_EXPANDS += \
./bsp/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
bsp/%.o: ../bsp/%.c
	@	riscv-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -DCore_V5F -I"/Users/litianyi/Desktop/flight_control" -I"/Users/litianyi/Desktop/flight_control/Debug" -I"/Users/litianyi/Desktop/flight_control/Core" -I"/Users/litianyi/Desktop/flight_control/User" -I"/Users/litianyi/Desktop/flight_control/Peripheral/inc" -I"/Users/litianyi/Desktop/flight_control/tasks/Inc" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/include" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/Common" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/GCC/RISC-V" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/MemMang" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

