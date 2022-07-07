################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/AS5600/as5600.c 

OBJS += \
./Lib/AS5600/as5600.o 

C_DEPS += \
./Lib/AS5600/as5600.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/AS5600/%.o Lib/AS5600/%.su: ../Lib/AS5600/%.c Lib/AS5600/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Lib -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-AS5600

clean-Lib-2f-AS5600:
	-$(RM) ./Lib/AS5600/as5600.d ./Lib/AS5600/as5600.o ./Lib/AS5600/as5600.su

.PHONY: clean-Lib-2f-AS5600

