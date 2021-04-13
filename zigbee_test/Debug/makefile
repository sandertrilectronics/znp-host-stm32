################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/znp-host-framework/framework/rpc/subdir.mk
-include Core/znp-host-framework/framework/platform/stm32/subdir.mk
-include Core/znp-host-framework/framework/mt/Zdo/subdir.mk
-include Core/znp-host-framework/framework/mt/Util/subdir.mk
-include Core/znp-host-framework/framework/mt/Sys/subdir.mk
-include Core/znp-host-framework/framework/mt/Sapi/subdir.mk
-include Core/znp-host-framework/framework/mt/AppCfg/subdir.mk
-include Core/znp-host-framework/framework/mt/Af/subdir.mk
-include Core/znp-host-framework/framework/mt/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
zigbee_test.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
zigbee_test.list \

OBJCOPY_BIN += \
zigbee_test.bin \


# All Target
all: zigbee_test.elf secondary-outputs

# Tool invocations
zigbee_test.elf: $(OBJS) $(USER_OBJS) C:\GitHub\znp-host-stm32\zigbee_test\STM32L496AGIX_FLASH.ld
	arm-none-eabi-gcc -o "zigbee_test.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\GitHub\znp-host-stm32\zigbee_test\STM32L496AGIX_FLASH.ld" --specs=nosys.specs -Wl,-Map="zigbee_test.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

zigbee_test.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "zigbee_test.list"
	@echo 'Finished building: $@'
	@echo ' '

zigbee_test.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "zigbee_test.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
