################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.c 

OBJS += \
./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o 

C_DEPS += \
./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SEGGER/Config/%.o ThirdParty/SEGGER/Config/%.su: ../ThirdParty/SEGGER/Config/%.c ThirdParty/SEGGER/Config/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I../Inc -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/ThirdParty/FreeRTOS/include" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/CMSIS-PeripheralDrivers/Inc" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/Core/Include" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/Device/ST/STM32F4xx/Include" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Prueba_SEGGER_PATCH/ThirdParty/SEGGER/SEGGER" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Prueba_SEGGER_PATCH/ThirdParty/SEGGER/Config" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Prueba_SEGGER_PATCH/ThirdParty/SEGGER/OS" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-SEGGER-2f-Config

clean-ThirdParty-2f-SEGGER-2f-Config:
	-$(RM) ./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d ./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o ./ThirdParty/SEGGER/Config/SEGGER_SYSVIEW_Config_FreeRTOS.su

.PHONY: clean-ThirdParty-2f-SEGGER-2f-Config

