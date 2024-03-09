################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/AdcDriver.c \
../Src/BasicTimer.c \
../Src/ExtiDriver.c \
../Src/GPIOxDriver.c \
../Src/I2CxDriver.c \
../Src/ILI9341.c \
../Src/LCDI2C.c \
../Src/PLLDriver.c \
../Src/PwmDriver.c \
../Src/RTCDriver.c \
../Src/SPIxDriver.c \
../Src/SysTick.c \
../Src/USARTxDriver.c 

OBJS += \
./Src/AdcDriver.o \
./Src/BasicTimer.o \
./Src/ExtiDriver.o \
./Src/GPIOxDriver.o \
./Src/I2CxDriver.o \
./Src/ILI9341.o \
./Src/LCDI2C.o \
./Src/PLLDriver.o \
./Src/PwmDriver.o \
./Src/RTCDriver.o \
./Src/SPIxDriver.o \
./Src/SysTick.o \
./Src/USARTxDriver.o 

C_DEPS += \
./Src/AdcDriver.d \
./Src/BasicTimer.d \
./Src/ExtiDriver.d \
./Src/GPIOxDriver.d \
./Src/I2CxDriver.d \
./Src/ILI9341.d \
./Src/LCDI2C.d \
./Src/PLLDriver.d \
./Src/PwmDriver.d \
./Src/RTCDriver.d \
./Src/SPIxDriver.d \
./Src/SysTick.d \
./Src/USARTxDriver.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I../Inc -I/home/daniel/Documents/CMSIS-repo/en.stm32cubef4_v1-27-0/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Core/Include -I/home/daniel/Documents/CMSIS-repo/en.stm32cubef4_v1-27-0/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/AdcDriver.d ./Src/AdcDriver.o ./Src/AdcDriver.su ./Src/BasicTimer.d ./Src/BasicTimer.o ./Src/BasicTimer.su ./Src/ExtiDriver.d ./Src/ExtiDriver.o ./Src/ExtiDriver.su ./Src/GPIOxDriver.d ./Src/GPIOxDriver.o ./Src/GPIOxDriver.su ./Src/I2CxDriver.d ./Src/I2CxDriver.o ./Src/I2CxDriver.su ./Src/ILI9341.d ./Src/ILI9341.o ./Src/ILI9341.su ./Src/LCDI2C.d ./Src/LCDI2C.o ./Src/LCDI2C.su ./Src/PLLDriver.d ./Src/PLLDriver.o ./Src/PLLDriver.su ./Src/PwmDriver.d ./Src/PwmDriver.o ./Src/PwmDriver.su ./Src/RTCDriver.d ./Src/RTCDriver.o ./Src/RTCDriver.su ./Src/SPIxDriver.d ./Src/SPIxDriver.o ./Src/SPIxDriver.su ./Src/SysTick.d ./Src/SysTick.o ./Src/SysTick.su ./Src/USARTxDriver.d ./Src/USARTxDriver.o ./Src/USARTxDriver.su

.PHONY: clean-Src

