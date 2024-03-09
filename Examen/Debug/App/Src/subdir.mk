################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/prueba2.c 

OBJS += \
./App/Src/prueba2.o 

C_DEPS += \
./App/Src/prueba2.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/daniel/Robotica/RTOS_workspace/Examen/Drivers/Core/Include" -I"/home/daniel/Robotica/RTOS_workspace/Examen/Drivers/Device/ST/STM32F4xx/Include" -I"/home/daniel/Robotica/RTOS_workspace/CMSIS-PeripheralDrivers/Inc" -I"/home/daniel/Robotica/RTOS_workspace/Examen/App/Inc" -I"/home/daniel/Robotica/RTOS_workspace/CMSIS-Full/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/prueba2.d ./App/Src/prueba2.o ./App/Src/prueba2.su

.PHONY: clean-App-2f-Src

