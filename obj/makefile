################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################
-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include tasks/Src/subdir.mk
-include modules/safety/subdir.mk
-include modules/navigation/subdir.mk
-include modules/estimator/subdir.mk
-include modules/control/subdir.mk
-include modules/comm/subdir.mk
-include drivers/sensors/subdir.mk
-include bsp/subdir.mk
-include app/subdir.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include FreeRTOS/portable/MemMang/subdir.mk
-include FreeRTOS/portable/GCC/RISC-V/subdir.mk
-include FreeRTOS/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
FlightControl_V5F.bin \
FlightControl_V5F.hex \

SECONDARY_LIST += \
FlightControl_V5F.lst \

SECONDARY_SIZE += \
FlightControl_V5F.siz \


# All Target
all: 
	$(MAKE) --no-print-directory main-build 

main-build: FlightControl_V5F.elf secondary-outputs

# Tool invocations
FlightControl_V5F.elf: $(OBJS) $(USER_OBJS_ESCAPE)
	@	riscv-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -T "/Users/litianyi/Desktop/flight_control/Ld/Link_v5f.ld" -nostartfiles -Xlinker --gc-sections -Xlinker --print-memory-usage -Wl,-Map,"FlightControl_V5F.map" --specs=nano.specs --specs=nosys.specs -o "FlightControl_V5F.elf" $(OBJS) $(LIBS)
FlightControl_V5F.bin: FlightControl_V5F.elf
	@	riscv-wch-elf-objcopy -O binary "FlightControl_V5F.elf" "FlightControl_V5F.bin"
FlightControl_V5F.hex: FlightControl_V5F.elf
	@	riscv-wch-elf-objcopy -O ihex "FlightControl_V5F.elf" "FlightControl_V5F.hex"
FlightControl_V5F.lst: FlightControl_V5F.elf
	@	riscv-wch-elf-objdump --all-headers --demangle --disassemble -M xw "FlightControl_V5F.elf" > "FlightControl_V5F.lst"
FlightControl_V5F.siz: FlightControl_V5F.elf
	riscv-wch-elf-size --format=berkeley "FlightControl_V5F.elf"

# Other Targets
clean:
	-$(RM) $(DIR_OBJS) $(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE) FlightControl_V5F.elf
	-$(RM) $(DIR_EXPANDS) $(CALLGRAPH_DOT)
	-$(RM) $(DIR_DEPS)
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets