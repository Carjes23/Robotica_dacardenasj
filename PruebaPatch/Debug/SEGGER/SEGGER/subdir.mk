################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SEGGER/SEGGER/SEGGER_RTT.c \
../SEGGER/SEGGER/SEGGER_RTT_printf.c \
../SEGGER/SEGGER/SEGGER_SYSVIEW.c 

S_UPPER_SRCS += \
../SEGGER/SEGGER/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./SEGGER/SEGGER/SEGGER_RTT.o \
./SEGGER/SEGGER/SEGGER_RTT_ASM_ARMv7M.o \
./SEGGER/SEGGER/SEGGER_RTT_printf.o \
./SEGGER/SEGGER/SEGGER_SYSVIEW.o 

S_UPPER_DEPS += \
./SEGGER/SEGGER/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./SEGGER/SEGGER/SEGGER_RTT.d \
./SEGGER/SEGGER/SEGGER_RTT_printf.d \
./SEGGER/SEGGER/SEGGER_SYSVIEW.d 


# Each subdirectory must supply rules for building sources it contributes
SEGGER/SEGGER/%.o SEGGER/SEGGER/%.su: ../SEGGER/SEGGER/%.c SEGGER/SEGGER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I../Inc -I"/home/daniel/Robotica/RTOS_workspace/Base_FreeRTOS/ThirdParty/FreeRTOS/include" -I"/home/daniel/Robotica/RTOS_workspace/Base_FreeRTOS/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"/home/daniel/Robotica/RTOS_workspace/CMSIS-PeripheralDrivers/Inc" -I"/home/daniel/Robotica/RTOS_workspace/Base_FreeRTOS/Core/Include" -I"/home/daniel/Robotica/RTOS_workspace/Base_FreeRTOS/Device/ST/STM32F4xx/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
SEGGER/SEGGER/%.o: ../SEGGER/SEGGER/%.S SEGGER/SEGGER/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-SEGGER-2f-SEGGER

clean-SEGGER-2f-SEGGER:
	-$(RM) ./SEGGER/SEGGER/SEGGER_RTT.d ./SEGGER/SEGGER/SEGGER_RTT.o ./SEGGER/SEGGER/SEGGER_RTT.su ./SEGGER/SEGGER/SEGGER_RTT_ASM_ARMv7M.d ./SEGGER/SEGGER/SEGGER_RTT_ASM_ARMv7M.o ./SEGGER/SEGGER/SEGGER_RTT_printf.d ./SEGGER/SEGGER/SEGGER_RTT_printf.o ./SEGGER/SEGGER/SEGGER_RTT_printf.su ./SEGGER/SEGGER/SEGGER_SYSVIEW.d ./SEGGER/SEGGER/SEGGER_SYSVIEW.o ./SEGGER/SEGGER/SEGGER_SYSVIEW.su

.PHONY: clean-SEGGER-2f-SEGGER

