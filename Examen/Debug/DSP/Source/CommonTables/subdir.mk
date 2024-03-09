################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DSP/Source/CommonTables/arm_common_tables.c 

OBJS += \
./DSP/Source/CommonTables/arm_common_tables.o 

C_DEPS += \
./DSP/Source/CommonTables/arm_common_tables.d 


# Each subdirectory must supply rules for building sources it contributes
DSP/Source/CommonTables/%.o DSP/Source/CommonTables/%.su: ../DSP/Source/CommonTables/%.c DSP/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/daniel/Documents/GitHub/TallerV/Examen/Drivers/Core/Include" -I"/home/daniel/Documents/GitHub/TallerV/Examen/Drivers/Device/ST/STM32F4xx/Include" -I"/home/daniel/Documents/GitHub/TallerV/CMSIS-PeripheralDrivers/Inc" -I"/home/daniel/Documents/GitHub/TallerV/Examen/App/Inc" -I"/home/daniel/Documents/GitHub/TallerV/Examen/DSP/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DSP-2f-Source-2f-CommonTables

clean-DSP-2f-Source-2f-CommonTables:
	-$(RM) ./DSP/Source/CommonTables/arm_common_tables.d ./DSP/Source/CommonTables/arm_common_tables.o ./DSP/Source/CommonTables/arm_common_tables.su

.PHONY: clean-DSP-2f-Source-2f-CommonTables

