################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/hdc1080/hdc1080.c 

OBJS += \
./Core/hdc1080/hdc1080.o 

C_DEPS += \
./Core/hdc1080/hdc1080.d 


# Each subdirectory must supply rules for building sources it contributes
Core/hdc1080/%.o Core/hdc1080/%.su Core/hdc1080/%.cyclo: ../Core/hdc1080/%.c Core/hdc1080/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/LE TUAN THANH/Desktop/Lora/Core/Lora" -I"C:/Users/LE TUAN THANH/Desktop/Lora/Core/hdc1080" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-hdc1080

clean-Core-2f-hdc1080:
	-$(RM) ./Core/hdc1080/hdc1080.cyclo ./Core/hdc1080/hdc1080.d ./Core/hdc1080/hdc1080.o ./Core/hdc1080/hdc1080.su

.PHONY: clean-Core-2f-hdc1080

