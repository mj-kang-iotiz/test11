################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/port.c 

OBJS += \
./third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/port.o 

C_DEPS += \
./third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/port.d 


# Each subdirectory must supply rules for building sources it contributes
third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/%.o third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/%.su third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/%.cyclo: ../third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/%.c third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include" -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable" -I"C:/Users/kang/Documents/test11/third_party/SystemView/SEGGER" -I"C:/Users/kang/Documents/test11/config" -I"C:/Users/kang/Documents/test11/lib/gps" -I"C:/Users/kang/Documents/test11/lib/gsm" -I"C:/Users/kang/Documents/test11/lib/parser" -I"C:/Users/kang/Documents/test11/lib/log" -I"C:/Users/kang/Documents/test11/lib/led" -I"C:/Users/kang/Documents/test11/lib/lora" -I"C:/Users/kang/Documents/test11/lib/rs485" -I"C:/Users/kang/Documents/test11/lib/ble" -I"C:/Users/kang/Documents/test11/app/gps" -I"C:/Users/kang/Documents/test11/app/gsm" -I"C:/Users/kang/Documents/test11/app/lora" -I"C:/Users/kang/Documents/test11/app/rs485" -I"C:/Users/kang/Documents/test11/app/ble" -I"C:/Users/kang/Documents/test11/app/params" -I"C:/Users/kang/Documents/test11/lib/utils/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-third_party-2f-FreeRTOS-2d-LTS-2f-FreeRTOS-2f-FreeRTOS-2d-Kernel-2f-portable

clean-third_party-2f-FreeRTOS-2d-LTS-2f-FreeRTOS-2f-FreeRTOS-2d-Kernel-2f-portable:
	-$(RM) ./third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/port.cyclo ./third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/port.d ./third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/port.o ./third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/port.su

.PHONY: clean-third_party-2f-FreeRTOS-2d-LTS-2f-FreeRTOS-2f-FreeRTOS-2d-Kernel-2f-portable

