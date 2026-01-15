################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/gps/gps.c \
../lib/gps/gps_nmea.c \
../lib/gps/gps_parser.c \
../lib/gps/gps_unicore.c \
../lib/gps/rtcm.c 

OBJS += \
./lib/gps/gps.o \
./lib/gps/gps_nmea.o \
./lib/gps/gps_parser.o \
./lib/gps/gps_unicore.o \
./lib/gps/rtcm.o 

C_DEPS += \
./lib/gps/gps.d \
./lib/gps/gps_nmea.d \
./lib/gps/gps_parser.d \
./lib/gps/gps_unicore.d \
./lib/gps/rtcm.d 


# Each subdirectory must supply rules for building sources it contributes
lib/gps/%.o lib/gps/%.su lib/gps/%.cyclo: ../lib/gps/%.c lib/gps/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include" -I"C:/Users/kang/Documents/test11/third_party/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable" -I"C:/Users/kang/Documents/test11/third_party/SystemView/SEGGER" -I"C:/Users/kang/Documents/test11/config" -I"C:/Users/kang/Documents/test11/lib/gps" -I"C:/Users/kang/Documents/test11/lib/gsm" -I"C:/Users/kang/Documents/test11/lib/parser" -I"C:/Users/kang/Documents/test11/lib/log" -I"C:/Users/kang/Documents/test11/lib/led" -I"C:/Users/kang/Documents/test11/lib/lora" -I"C:/Users/kang/Documents/test11/lib/rs485" -I"C:/Users/kang/Documents/test11/lib/ble" -I"C:/Users/kang/Documents/test11/app/gps" -I"C:/Users/kang/Documents/test11/app/gsm" -I"C:/Users/kang/Documents/test11/app/lora" -I"C:/Users/kang/Documents/test11/app/rs485" -I"C:/Users/kang/Documents/test11/app/ble" -I"C:/Users/kang/Documents/test11/app/params" -I"C:/Users/kang/Documents/test11/lib/utils/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-gps

clean-lib-2f-gps:
	-$(RM) ./lib/gps/gps.cyclo ./lib/gps/gps.d ./lib/gps/gps.o ./lib/gps/gps.su ./lib/gps/gps_nmea.cyclo ./lib/gps/gps_nmea.d ./lib/gps/gps_nmea.o ./lib/gps/gps_nmea.su ./lib/gps/gps_parser.cyclo ./lib/gps/gps_parser.d ./lib/gps/gps_parser.o ./lib/gps/gps_parser.su ./lib/gps/gps_unicore.cyclo ./lib/gps/gps_unicore.d ./lib/gps/gps_unicore.o ./lib/gps/gps_unicore.su ./lib/gps/rtcm.cyclo ./lib/gps/rtcm.d ./lib/gps/rtcm.o ./lib/gps/rtcm.su

.PHONY: clean-lib-2f-gps

