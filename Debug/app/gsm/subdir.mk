################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../app/gsm/gsm_app.c \
../app/gsm/gsm_port.c \
../app/gsm/lte_init.c \
../app/gsm/ntrip_app.c 

OBJS += \
./app/gsm/gsm_app.o \
./app/gsm/gsm_port.o \
./app/gsm/lte_init.o \
./app/gsm/ntrip_app.o 

C_DEPS += \
./app/gsm/gsm_app.d \
./app/gsm/gsm_port.d \
./app/gsm/lte_init.d \
./app/gsm/ntrip_app.d 


# Each subdirectory must supply rules for building sources it contributes
app/gsm/%.o app/gsm/%.su app/gsm/%.cyclo: ../app/gsm/%.c app/gsm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include" -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable" -I"C:/Users/kang/Documents/test11/third_party/SystemView/SEGGER" -I"C:/Users/kang/Documents/test11/config" -I"C:/Users/kang/Documents/test11/lib/gps" -I"C:/Users/kang/Documents/test11/lib/gsm" -I"C:/Users/kang/Documents/test11/lib/parser" -I"C:/Users/kang/Documents/test11/lib/log" -I"C:/Users/kang/Documents/test11/lib/led" -I"C:/Users/kang/Documents/test11/lib/lora" -I"C:/Users/kang/Documents/test11/lib/rs485" -I"C:/Users/kang/Documents/test11/lib/ble" -I"C:/Users/kang/Documents/test11/app/gps" -I"C:/Users/kang/Documents/test11/app/gsm" -I"C:/Users/kang/Documents/test11/app/lora" -I"C:/Users/kang/Documents/test11/app/rs485" -I"C:/Users/kang/Documents/test11/app/ble" -I"C:/Users/kang/Documents/test11/app/params" -I"C:/Users/kang/Documents/test11/lib/utils/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-app-2f-gsm

clean-app-2f-gsm:
	-$(RM) ./app/gsm/gsm_app.cyclo ./app/gsm/gsm_app.d ./app/gsm/gsm_app.o ./app/gsm/gsm_app.su ./app/gsm/gsm_port.cyclo ./app/gsm/gsm_port.d ./app/gsm/gsm_port.o ./app/gsm/gsm_port.su ./app/gsm/lte_init.cyclo ./app/gsm/lte_init.d ./app/gsm/lte_init.o ./app/gsm/lte_init.su ./app/gsm/ntrip_app.cyclo ./app/gsm/ntrip_app.d ./app/gsm/ntrip_app.o ./app/gsm/ntrip_app.su

.PHONY: clean-app-2f-gsm

