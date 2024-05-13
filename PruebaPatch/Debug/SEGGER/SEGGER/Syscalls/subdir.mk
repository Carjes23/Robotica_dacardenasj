################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c \
../SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.c \
../SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.c \
../SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.c 

OBJS += \
./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o \
./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.o \
./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.o \
./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.o 

C_DEPS += \
./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d \
./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.d \
./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.d \
./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.d 


# Each subdirectory must supply rules for building sources it contributes
SEGGER/SEGGER/Syscalls/%.o SEGGER/SEGGER/Syscalls/%.su: ../SEGGER/SEGGER/Syscalls/%.c SEGGER/SEGGER/Syscalls/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I../Inc -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/ThirdParty/FreeRTOS/include" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/CMSIS-PeripheralDrivers/Inc" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/Core/Include" -I"C:/Users/DANIE/OneDrive/Documents/GitHub/Robotica_dacardenasj/Base_FreeRTOS/Device/ST/STM32F4xx/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-SEGGER-2f-SEGGER-2f-Syscalls

clean-SEGGER-2f-SEGGER-2f-Syscalls:
	-$(RM) ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.su ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.d ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.o ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.su ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.d ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.o ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.su ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.d ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.o ./SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.su

.PHONY: clean-SEGGER-2f-SEGGER-2f-Syscalls

