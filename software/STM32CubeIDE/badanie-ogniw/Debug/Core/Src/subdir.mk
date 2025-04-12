################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BMXX80.c \
../Core/Src/INA219.c \
../Core/Src/fatfs_sd.c \
../Core/Src/fonts.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/printf_to_uart.c \
../Core/Src/sensirion_common.c \
../Core/Src/sensirion_configuration.c \
../Core/Src/sgp30.c \
../Core/Src/sgp30_featureset.c \
../Core/Src/spi.c \
../Core/Src/st7735.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c \
../Core/Src/usart.c \
../Core/Src/usb_otg.c 

OBJS += \
./Core/Src/BMXX80.o \
./Core/Src/INA219.o \
./Core/Src/fatfs_sd.o \
./Core/Src/fonts.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/printf_to_uart.o \
./Core/Src/sensirion_common.o \
./Core/Src/sensirion_configuration.o \
./Core/Src/sgp30.o \
./Core/Src/sgp30_featureset.o \
./Core/Src/spi.o \
./Core/Src/st7735.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o \
./Core/Src/usart.o \
./Core/Src/usb_otg.o 

C_DEPS += \
./Core/Src/BMXX80.d \
./Core/Src/INA219.d \
./Core/Src/fatfs_sd.d \
./Core/Src/fonts.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/printf_to_uart.d \
./Core/Src/sensirion_common.d \
./Core/Src/sensirion_configuration.d \
./Core/Src/sgp30.d \
./Core/Src/sgp30_featureset.d \
./Core/Src/spi.d \
./Core/Src/st7735.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d \
./Core/Src/usart.d \
./Core/Src/usb_otg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BMXX80.cyclo ./Core/Src/BMXX80.d ./Core/Src/BMXX80.o ./Core/Src/BMXX80.su ./Core/Src/INA219.cyclo ./Core/Src/INA219.d ./Core/Src/INA219.o ./Core/Src/INA219.su ./Core/Src/fatfs_sd.cyclo ./Core/Src/fatfs_sd.d ./Core/Src/fatfs_sd.o ./Core/Src/fatfs_sd.su ./Core/Src/fonts.cyclo ./Core/Src/fonts.d ./Core/Src/fonts.o ./Core/Src/fonts.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/printf_to_uart.cyclo ./Core/Src/printf_to_uart.d ./Core/Src/printf_to_uart.o ./Core/Src/printf_to_uart.su ./Core/Src/sensirion_common.cyclo ./Core/Src/sensirion_common.d ./Core/Src/sensirion_common.o ./Core/Src/sensirion_common.su ./Core/Src/sensirion_configuration.cyclo ./Core/Src/sensirion_configuration.d ./Core/Src/sensirion_configuration.o ./Core/Src/sensirion_configuration.su ./Core/Src/sgp30.cyclo ./Core/Src/sgp30.d ./Core/Src/sgp30.o ./Core/Src/sgp30.su ./Core/Src/sgp30_featureset.cyclo ./Core/Src/sgp30_featureset.d ./Core/Src/sgp30_featureset.o ./Core/Src/sgp30_featureset.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/st7735.cyclo ./Core/Src/st7735.d ./Core/Src/st7735.o ./Core/Src/st7735.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/usb_otg.cyclo ./Core/Src/usb_otg.d ./Core/Src/usb_otg.o ./Core/Src/usb_otg.su

.PHONY: clean-Core-2f-Src

