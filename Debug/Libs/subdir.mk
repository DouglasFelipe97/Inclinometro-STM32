################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libs/OLED.c \
../Libs/OLED_Fonst.c \
../Libs/OLED_Icons.c \
../Libs/mpu6050.c 

OBJS += \
./Libs/OLED.o \
./Libs/OLED_Fonst.o \
./Libs/OLED_Icons.o \
./Libs/mpu6050.o 

C_DEPS += \
./Libs/OLED.d \
./Libs/OLED_Fonst.d \
./Libs/OLED_Icons.d \
./Libs/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
Libs/%.o Libs/%.su Libs/%.cyclo: ../Libs/%.c Libs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Laboratorio/Desktop/Douglas Autoban/STM32Cube/Inclinometro/Libs" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Libs

clean-Libs:
	-$(RM) ./Libs/OLED.cyclo ./Libs/OLED.d ./Libs/OLED.o ./Libs/OLED.su ./Libs/OLED_Fonst.cyclo ./Libs/OLED_Fonst.d ./Libs/OLED_Fonst.o ./Libs/OLED_Fonst.su ./Libs/OLED_Icons.cyclo ./Libs/OLED_Icons.d ./Libs/OLED_Icons.o ./Libs/OLED_Icons.su ./Libs/mpu6050.cyclo ./Libs/mpu6050.d ./Libs/mpu6050.o ./Libs/mpu6050.su

.PHONY: clean-Libs

