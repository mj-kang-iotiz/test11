################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../third_party/SystemView/SEGGER/SEGGER_RTT.c \
../third_party/SystemView/SEGGER/SEGGER_RTT_printf.c \
../third_party/SystemView/SEGGER/SEGGER_SYSVIEW.c \
../third_party/SystemView/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.c \
../third_party/SystemView/SEGGER/SEGGER_SYSVIEW_FreeRTOS.c 

S_UPPER_SRCS += \
../third_party/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./third_party/SystemView/SEGGER/SEGGER_RTT.o \
./third_party/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.o \
./third_party/SystemView/SEGGER/SEGGER_RTT_printf.o \
./third_party/SystemView/SEGGER/SEGGER_SYSVIEW.o \
./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.o \
./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_FreeRTOS.o 

S_UPPER_DEPS += \
./third_party/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./third_party/SystemView/SEGGER/SEGGER_RTT.d \
./third_party/SystemView/SEGGER/SEGGER_RTT_printf.d \
./third_party/SystemView/SEGGER/SEGGER_SYSVIEW.d \
./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.d \
./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
third_party/SystemView/SEGGER/%.o third_party/SystemView/SEGGER/%.su third_party/SystemView/SEGGER/%.cyclo: ../third_party/SystemView/SEGGER/%.c third_party/SystemView/SEGGER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include" -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable" -I"C:/Users/kang/Documents/test11/third_party/SystemView/SEGGER" -I"C:/Users/kang/Documents/test11/config" -I"C:/Users/kang/Documents/test11/lib/gps" -I"C:/Users/kang/Documents/test11/lib/gsm" -I"C:/Users/kang/Documents/test11/lib/parser" -I"C:/Users/kang/Documents/test11/lib/log" -I"C:/Users/kang/Documents/test11/lib/led" -I"C:/Users/kang/Documents/test11/lib/lora" -I"C:/Users/kang/Documents/test11/lib/rs485" -I"C:/Users/kang/Documents/test11/lib/ble" -I"C:/Users/kang/Documents/test11/app/gps" -I"C:/Users/kang/Documents/test11/app/gsm" -I"C:/Users/kang/Documents/test11/app/lora" -I"C:/Users/kang/Documents/test11/app/rs485" -I"C:/Users/kang/Documents/test11/app/ble" -I"C:/Users/kang/Documents/test11/app/params" -I"C:/Users/kang/Documents/test11/lib/utils/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
third_party/SystemView/SEGGER/%.o: ../third_party/SystemView/SEGGER/%.S third_party/SystemView/SEGGER/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable" -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel" -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-third_party-2f-SystemView-2f-SEGGER

clean-third_party-2f-SystemView-2f-SEGGER:
	-$(RM) ./third_party/SystemView/SEGGER/SEGGER_RTT.cyclo ./third_party/SystemView/SEGGER/SEGGER_RTT.d ./third_party/SystemView/SEGGER/SEGGER_RTT.o ./third_party/SystemView/SEGGER/SEGGER_RTT.su ./third_party/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.d ./third_party/SystemView/SEGGER/SEGGER_RTT_ASM_ARMv7M.o ./third_party/SystemView/SEGGER/SEGGER_RTT_printf.cyclo ./third_party/SystemView/SEGGER/SEGGER_RTT_printf.d ./third_party/SystemView/SEGGER/SEGGER_RTT_printf.o ./third_party/SystemView/SEGGER/SEGGER_RTT_printf.su ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW.cyclo ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW.d ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW.o ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW.su ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.cyclo ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.d ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.o ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_Config_FreeRTOS.su ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_FreeRTOS.cyclo ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_FreeRTOS.d ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_FreeRTOS.o ./third_party/SystemView/SEGGER/SEGGER_SYSVIEW_FreeRTOS.su

.PHONY: clean-third_party-2f-SystemView-2f-SEGGER

