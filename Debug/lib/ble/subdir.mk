################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/ble/ble.c \
../lib/ble/ble_parser.c \
../lib/ble/ble_task.c 

OBJS += \
./lib/ble/ble.o \
./lib/ble/ble_parser.o \
./lib/ble/ble_task.o 

C_DEPS += \
./lib/ble/ble.d \
./lib/ble/ble_parser.d \
./lib/ble/ble_task.d 


# Each subdirectory must supply rules for building sources it contributes
lib/ble/%.o lib/ble/%.su lib/ble/%.cyclo: ../lib/ble/%.c lib/ble/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include" -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable" -I"C:/Users/kang/Documents/test11/third_party/SystemView/SEGGER" -I"C:/Users/kang/Documents/test11/config" -I"C:/Users/kang/Documents/test11/lib/gps" -I"C:/Users/kang/Documents/test11/lib/gsm" -I"C:/Users/kang/Documents/test11/lib/parser" -I"C:/Users/kang/Documents/test11/lib/log" -I"C:/Users/kang/Documents/test11/lib/led" -I"C:/Users/kang/Documents/test11/lib/lora" -I"C:/Users/kang/Documents/test11/lib/rs485" -I"C:/Users/kang/Documents/test11/lib/ble" -I"C:/Users/kang/Documents/test11/app/gps" -I"C:/Users/kang/Documents/test11/app/gsm" -I"C:/Users/kang/Documents/test11/app/lora" -I"C:/Users/kang/Documents/test11/app/rs485" -I"C:/Users/kang/Documents/test11/app/ble" -I"C:/Users/kang/Documents/test11/app/params" -I"C:/Users/kang/Documents/test11/lib/utils/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-ble

clean-lib-2f-ble:
	-$(RM) ./lib/ble/ble.cyclo ./lib/ble/ble.d ./lib/ble/ble.o ./lib/ble/ble.su ./lib/ble/ble_parser.cyclo ./lib/ble/ble_parser.d ./lib/ble/ble_parser.o ./lib/ble/ble_parser.su ./lib/ble/ble_task.cyclo ./lib/ble/ble_task.d ./lib/ble/ble_task.o ./lib/ble/ble_task.su

.PHONY: clean-lib-2f-ble

