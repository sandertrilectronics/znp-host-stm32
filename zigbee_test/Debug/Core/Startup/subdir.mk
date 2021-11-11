################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/startup/startup_stm32l496xx.s 

OBJS += \
./Core/startup/startup_stm32l496xx.o 

S_DEPS += \
./Core/startup/startup_stm32l496xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/startup/%.o: ../Core/startup/%.s Core/startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I../Core/startup -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

