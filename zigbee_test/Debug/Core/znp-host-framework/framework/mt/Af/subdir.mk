################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/znp-host-framework/framework/mt/Af/mtAf.c 

OBJS += \
./Core/znp-host-framework/framework/mt/Af/mtAf.o 

C_DEPS += \
./Core/znp-host-framework/framework/mt/Af/mtAf.d 


# Each subdirectory must supply rules for building sources it contributes
Core/znp-host-framework/framework/mt/Af/mtAf.o: ../Core/znp-host-framework/framework/mt/Af/mtAf.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L496xx -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Core/znp-host-framework/framework/mt/Sapi -I../Core/znp-host-framework/framework/platform/stm32 -I../Core/znp-host-framework/framework/mt/Sys -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Core/znp-host-framework/framework/mt/Af -I../Drivers/CMSIS/Include -I../Core/znp-host-framework/framework/mt -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Core/znp-host-framework/framework/mt/Zdo -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Core/znp-host-framework/framework/rpc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/znp-host-framework/framework/mt/Af/mtAf.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

