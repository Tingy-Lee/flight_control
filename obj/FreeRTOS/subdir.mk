################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS/croutine.c \
../FreeRTOS/event_groups.c \
../FreeRTOS/list.c \
../FreeRTOS/queue.c \
../FreeRTOS/stream_buffer.c \
../FreeRTOS/tasks.c \
../FreeRTOS/timers.c 

C_DEPS += \
./FreeRTOS/croutine.d \
./FreeRTOS/event_groups.d \
./FreeRTOS/list.d \
./FreeRTOS/queue.d \
./FreeRTOS/stream_buffer.d \
./FreeRTOS/tasks.d \
./FreeRTOS/timers.d 

OBJS += \
./FreeRTOS/croutine.o \
./FreeRTOS/event_groups.o \
./FreeRTOS/list.o \
./FreeRTOS/queue.o \
./FreeRTOS/stream_buffer.o \
./FreeRTOS/tasks.o \
./FreeRTOS/timers.o 

DIR_OBJS += \
./FreeRTOS/*.o \

DIR_DEPS += \
./FreeRTOS/*.d \

DIR_EXPANDS += \
./FreeRTOS/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/%.o: ../FreeRTOS/%.c
	@	riscv-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -DCore_V5F -I"/Users/litianyi/Desktop/flight_control" -I"/Users/litianyi/Desktop/flight_control/Debug" -I"/Users/litianyi/Desktop/flight_control/Core" -I"/Users/litianyi/Desktop/flight_control/User" -I"/Users/litianyi/Desktop/flight_control/Peripheral/inc" -I"/Users/litianyi/Desktop/flight_control/tasks/Inc" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/include" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/Common" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/GCC/RISC-V" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions" -I"/Users/litianyi/Desktop/flight_control/FreeRTOS/portable/MemMang" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

