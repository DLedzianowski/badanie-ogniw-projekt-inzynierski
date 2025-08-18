################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/sensors/BMXX80.c \
../Core/Src/sensors/INA219.c \
../Core/Src/sensors/fatfs_sd.c \
../Core/Src/sensors/fonts.c \
../Core/Src/sensors/printf_to_uart.c \
../Core/Src/sensors/sensirion_common.c \
../Core/Src/sensors/sensirion_configuration.c \
../Core/Src/sensors/sgp30.c \
../Core/Src/sensors/sgp30_featureset.c \
../Core/Src/sensors/st7735.c 

OBJS += \
./Core/Src/sensors/BMXX80.o \
./Core/Src/sensors/INA219.o \
./Core/Src/sensors/fatfs_sd.o \
./Core/Src/sensors/fonts.o \
./Core/Src/sensors/printf_to_uart.o \
./Core/Src/sensors/sensirion_common.o \
./Core/Src/sensors/sensirion_configuration.o \
./Core/Src/sensors/sgp30.o \
./Core/Src/sensors/sgp30_featureset.o \
./Core/Src/sensors/st7735.o 

C_DEPS += \
./Core/Src/sensors/BMXX80.d \
./Core/Src/sensors/INA219.d \
./Core/Src/sensors/fatfs_sd.d \
./Core/Src/sensors/fonts.d \
./Core/Src/sensors/printf_to_uart.d \
./Core/Src/sensors/sensirion_common.d \
./Core/Src/sensors/sensirion_configuration.d \
./Core/Src/sensors/sgp30.d \
./Core/Src/sensors/sgp30_featureset.d \
./Core/Src/sensors/st7735.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/sensors/%.o Core/Src/sensors/%.su Core/Src/sensors/%.cyclo: ../Core/Src/sensors/%.c Core/Src/sensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-sensors

clean-Core-2f-Src-2f-sensors:
	-$(RM) ./Core/Src/sensors/BMXX80.cyclo ./Core/Src/sensors/BMXX80.d ./Core/Src/sensors/BMXX80.o ./Core/Src/sensors/BMXX80.su ./Core/Src/sensors/INA219.cyclo ./Core/Src/sensors/INA219.d ./Core/Src/sensors/INA219.o ./Core/Src/sensors/INA219.su ./Core/Src/sensors/fatfs_sd.cyclo ./Core/Src/sensors/fatfs_sd.d ./Core/Src/sensors/fatfs_sd.o ./Core/Src/sensors/fatfs_sd.su ./Core/Src/sensors/fonts.cyclo ./Core/Src/sensors/fonts.d ./Core/Src/sensors/fonts.o ./Core/Src/sensors/fonts.su ./Core/Src/sensors/printf_to_uart.cyclo ./Core/Src/sensors/printf_to_uart.d ./Core/Src/sensors/printf_to_uart.o ./Core/Src/sensors/printf_to_uart.su ./Core/Src/sensors/sensirion_common.cyclo ./Core/Src/sensors/sensirion_common.d ./Core/Src/sensors/sensirion_common.o ./Core/Src/sensors/sensirion_common.su ./Core/Src/sensors/sensirion_configuration.cyclo ./Core/Src/sensors/sensirion_configuration.d ./Core/Src/sensors/sensirion_configuration.o ./Core/Src/sensors/sensirion_configuration.su ./Core/Src/sensors/sgp30.cyclo ./Core/Src/sensors/sgp30.d ./Core/Src/sensors/sgp30.o ./Core/Src/sensors/sgp30.su ./Core/Src/sensors/sgp30_featureset.cyclo ./Core/Src/sensors/sgp30_featureset.d ./Core/Src/sensors/sgp30_featureset.o ./Core/Src/sensors/sgp30_featureset.su ./Core/Src/sensors/st7735.cyclo ./Core/Src/sensors/st7735.d ./Core/Src/sensors/st7735.o ./Core/Src/sensors/st7735.su

.PHONY: clean-Core-2f-Src-2f-sensors

