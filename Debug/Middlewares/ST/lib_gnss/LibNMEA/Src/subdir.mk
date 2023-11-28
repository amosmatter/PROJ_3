################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/lib_gnss/LibNMEA/Src/NMEA_parser.c 

OBJS += \
./Middlewares/ST/lib_gnss/LibNMEA/Src/NMEA_parser.o 

C_DEPS += \
./Middlewares/ST/lib_gnss/LibNMEA/Src/NMEA_parser.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/lib_gnss/LibNMEA/Src/%.o Middlewares/ST/lib_gnss/LibNMEA/Src/%.su Middlewares/ST/lib_gnss/LibNMEA/Src/%.cyclo: ../Middlewares/ST/lib_gnss/LibNMEA/Src/%.c Middlewares/ST/lib_gnss/LibNMEA/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -I../Lib -I../lib_gnss/Target -I../Middlewares/ST/lib_gnss/LibGNSS/Inc -I../Middlewares/ST/lib_gnss/LibNMEA/Inc -I../Middlewares/ST/lib_gnss/Template -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-lib_gnss-2f-LibNMEA-2f-Src

clean-Middlewares-2f-ST-2f-lib_gnss-2f-LibNMEA-2f-Src:
	-$(RM) ./Middlewares/ST/lib_gnss/LibNMEA/Src/NMEA_parser.cyclo ./Middlewares/ST/lib_gnss/LibNMEA/Src/NMEA_parser.d ./Middlewares/ST/lib_gnss/LibNMEA/Src/NMEA_parser.o ./Middlewares/ST/lib_gnss/LibNMEA/Src/NMEA_parser.su

.PHONY: clean-Middlewares-2f-ST-2f-lib_gnss-2f-LibNMEA-2f-Src

