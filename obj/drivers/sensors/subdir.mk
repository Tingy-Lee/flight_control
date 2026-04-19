################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/sensors/barometer.c \
../drivers/sensors/gps.c \
../drivers/sensors/imu.c \
../drivers/sensors/rc_input.c 

C_DEPS += \
./drivers/sensors/barometer.d \
./drivers/sensors/gps.d \
./drivers/sensors/imu.d \
./drivers/sensors/rc_input.d 

OBJS += \
./drivers/sensors/barometer.o \
./drivers/sensors/gps.o \
./drivers/sensors/imu.o \
./drivers/sensors/rc_input.o 

DIR_OBJS += \
./drivers/sensors/*.o \

DIR_DEPS += \
./drivers/sensors/*.d \

DIR_EXPANDS += \
./drivers/sensors/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
drivers/sensors/%.o: ../drivers/sensors/%.c
	@	riscv-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -DCore_V5F -I"/Users/litianyi/Desktop/flight_control" -I"/Users/litianyi/Desktop/flight_control/Debug" -I"/Users/litianyi/Desktop/flight_control/Core" -I"/Users/litianyi/Desktop/flight_control/User" -I"/Users/litianyi/Desktop/flight_control/Peripheral/inc" -I"/Users/litianyi/Desktop/flight_control/tasks/Inc" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/include" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/Common" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/GCC/RISC-V" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/MemMang" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

