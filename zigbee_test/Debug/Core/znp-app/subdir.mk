################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/znp-app/znp_cb.c \
../Core/znp-app/znp_cmd.c \
../Core/znp-app/znp_if.c 

OBJS += \
./Core/znp-app/znp_cb.o \
./Core/znp-app/znp_cmd.o \
./Core/znp-app/znp_if.o 

C_DEPS += \
./Core/znp-app/znp_cb.d \
./Core/znp-app/znp_cmd.d \
./Core/znp-app/znp_if.d 


# Each subdirectory must supply rules for building sources it contributes
Core/znp-app/%.o: ../Core/znp-app/%.c Core/znp-app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Core/Inc -I../Core/znp-app -I../Core/znp-host-framework/framework/rpc -I../Core/znp-host-framework/framework/platform/stm32 -I../Core/znp-host-framework/framework/mt/Af -I../Core/znp-host-framework/framework/mt/AppCfg -I../Core/znp-host-framework/framework/mt/Sapi -I../Core/znp-host-framework/framework/mt/Sys -I../Core/znp-host-framework/framework/mt/Util -I../Core/znp-host-framework/framework/mt/Zdo -I../Core/znp-host-framework/framework/mt -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

