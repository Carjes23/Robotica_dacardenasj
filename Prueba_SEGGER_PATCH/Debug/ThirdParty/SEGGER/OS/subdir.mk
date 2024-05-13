################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.c 

OBJS += \
./ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.o 

C_DEPS += \
./ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SEGGER/OS/%.o ThirdParty/SEGGER/OS/%.su: ../ThirdParty/SEGGER/OS/%.c ThirdParty/SEGGER/OS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I../Inc -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/ThirdParty/FreeRTOS/include" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/CMSIS-PeripheralDrivers/Inc" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/Core/Include" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/Device/ST/STM32F4xx/Include" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Prueba_SEGGER_PATCH/ThirdParty/SEGGER/SEGGER" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Prueba_SEGGER_PATCH/ThirdParty/SEGGER/Config" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Prueba_SEGGER_PATCH/ThirdParty/SEGGER/OS" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-SEGGER-2f-OS

clean-ThirdParty-2f-SEGGER-2f-OS:
	-$(RM) ./ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.d ./ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.o ./ThirdParty/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.su

.PHONY: clean-ThirdParty-2f-SEGGER-2f-OS

