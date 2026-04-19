################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tasks/Src/commander_task.c \
../tasks/Src/control_task.c \
../tasks/Src/initial_task.c \
../tasks/Src/logger_task.c \
../tasks/Src/sensor_task.c 

C_DEPS += \
./tasks/Src/commander_task.d \
./tasks/Src/control_task.d \
./tasks/Src/initial_task.d \
./tasks/Src/logger_task.d \
./tasks/Src/sensor_task.d 

OBJS += \
./tasks/Src/commander_task.o \
./tasks/Src/control_task.o \
./tasks/Src/initial_task.o \
./tasks/Src/logger_task.o \
./tasks/Src/sensor_task.o 

DIR_OBJS += \
./tasks/Src/*.o \

DIR_DEPS += \
./tasks/Src/*.d \

DIR_EXPANDS += \
./tasks/Src/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
tasks/Src/%.o: ../tasks/Src/%.c
	@	riscv-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -DCore_V5F -I"/Users/litianyi/Desktop/flight_control" -I"/Users/litianyi/Desktop/flight_control/Debug" -I"/Users/litianyi/Desktop/flight_control/Core" -I"/Users/litianyi/Desktop/flight_control/User" -I"/Users/litianyi/Desktop/flight_control/Peripheral/inc" -I"/Users/litianyi/Desktop/flight_control/tasks/Inc" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/include" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/Common" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/GCC/RISC-V" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/MemMang" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

