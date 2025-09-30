################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/logic/user_OLED.c \
../Core/Src/logic/user_SDcard.c \
../Core/Src/logic/user_fnc.c 

OBJS += \
./Core/Src/logic/user_OLED.o \
./Core/Src/logic/user_SDcard.o \
./Core/Src/logic/user_fnc.o 

C_DEPS += \
./Core/Src/logic/user_OLED.d \
./Core/Src/logic/user_SDcard.d \
./Core/Src/logic/user_fnc.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/logic/%.o Core/Src/logic/%.su Core/Src/logic/%.cyclo: ../Core/Src/logic/%.c Core/Src/logic/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/idomi/OneDrive/Dokumenty/GitHub/badanie-ogniw-projekt-przejsciowy/software/STM32CubeIDE/badanie-ogniw-blackpill/Core" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-logic

clean-Core-2f-Src-2f-logic:
	-$(RM) ./Core/Src/logic/user_OLED.cyclo ./Core/Src/logic/user_OLED.d ./Core/Src/logic/user_OLED.o ./Core/Src/logic/user_OLED.su ./Core/Src/logic/user_SDcard.cyclo ./Core/Src/logic/user_SDcard.d ./Core/Src/logic/user_SDcard.o ./Core/Src/logic/user_SDcard.su ./Core/Src/logic/user_fnc.cyclo ./Core/Src/logic/user_fnc.d ./Core/Src/logic/user_fnc.o ./Core/Src/logic/user_fnc.su

.PHONY: clean-Core-2f-Src-2f-logic

