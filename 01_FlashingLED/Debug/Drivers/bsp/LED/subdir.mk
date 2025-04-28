################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/bsp/LED/led.c 

OBJS += \
./Drivers/bsp/LED/led.o 

C_DEPS += \
./Drivers/bsp/LED/led.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/bsp/LED/%.o Drivers/bsp/LED/%.su Drivers/bsp/LED/%.cyclo: ../Drivers/bsp/LED/%.c Drivers/bsp/LED/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-bsp-2f-LED

clean-Drivers-2f-bsp-2f-LED:
	-$(RM) ./Drivers/bsp/LED/led.cyclo ./Drivers/bsp/LED/led.d ./Drivers/bsp/LED/led.o ./Drivers/bsp/LED/led.su

.PHONY: clean-Drivers-2f-bsp-2f-LED

