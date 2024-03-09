################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c \
../ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.c \
../ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.c \
../ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.c 

OBJS += \
./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o \
./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.o \
./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.o \
./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.o 

C_DEPS += \
./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d \
./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.d \
./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.d \
./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SEGGER/SEGGER/Syscalls/%.o ThirdParty/SEGGER/SEGGER/Syscalls/%.su: ../ThirdParty/SEGGER/SEGGER/Syscalls/%.c ThirdParty/SEGGER/SEGGER/Syscalls/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I../Inc -I"/home/daniel/Robotica/RTOS_workspace/Base_FreeRTOS/ThirdParty/FreeRTOS/include" -I"/home/daniel/Robotica/RTOS_workspace/Base_FreeRTOS/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/daniel/Robotica/RTOS_workspace/CMSIS-PeripheralDrivers/Inc" -I"/home/daniel/Robotica/RTOS_workspace/Base_FreeRTOS/Core/Include" -I"/home/daniel/Robotica/RTOS_workspace/Base_FreeRTOS/Device/ST/STM32F4xx/Include" -I"/home/daniel/Robotica/RTOS_workspace/Prueba_SEGGER_PATCH/ThirdParty/SEGGER/SEGGER" -I"/home/daniel/Robotica/RTOS_workspace/Prueba_SEGGER_PATCH/ThirdParty/SEGGER/Config" -I"/home/daniel/Robotica/RTOS_workspace/Prueba_SEGGER_PATCH/ThirdParty/SEGGER/OS" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-SEGGER-2f-SEGGER-2f-Syscalls

clean-ThirdParty-2f-SEGGER-2f-SEGGER-2f-Syscalls:
	-$(RM) ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.su ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.d ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.o ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.su ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.d ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.o ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.su ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.d ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.o ./ThirdParty/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.su

.PHONY: clean-ThirdParty-2f-SEGGER-2f-SEGGER-2f-Syscalls

