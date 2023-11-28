################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/minmea-master/minmea.c 

OBJS += \
./Lib/minmea-master/minmea.o 

C_DEPS += \
./Lib/minmea-master/minmea.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/minmea-master/%.o Lib/minmea-master/%.su Lib/minmea-master/%.cyclo: ../Lib/minmea-master/%.c Lib/minmea-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -I../Lib -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-minmea-2d-master

clean-Lib-2f-minmea-2d-master:
	-$(RM) ./Lib/minmea-master/minmea.cyclo ./Lib/minmea-master/minmea.d ./Lib/minmea-master/minmea.o ./Lib/minmea-master/minmea.su

.PHONY: clean-Lib-2f-minmea-2d-master

