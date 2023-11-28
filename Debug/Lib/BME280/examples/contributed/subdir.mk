################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/BME280/examples/contributed/bsd_userspace.c \
../Lib/BME280/examples/contributed/linux_userspace.c 

OBJS += \
./Lib/BME280/examples/contributed/bsd_userspace.o \
./Lib/BME280/examples/contributed/linux_userspace.o 

C_DEPS += \
./Lib/BME280/examples/contributed/bsd_userspace.d \
./Lib/BME280/examples/contributed/linux_userspace.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/BME280/examples/contributed/%.o Lib/BME280/examples/contributed/%.su Lib/BME280/examples/contributed/%.cyclo: ../Lib/BME280/examples/contributed/%.c Lib/BME280/examples/contributed/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -I../Lib -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-BME280-2f-examples-2f-contributed

clean-Lib-2f-BME280-2f-examples-2f-contributed:
	-$(RM) ./Lib/BME280/examples/contributed/bsd_userspace.cyclo ./Lib/BME280/examples/contributed/bsd_userspace.d ./Lib/BME280/examples/contributed/bsd_userspace.o ./Lib/BME280/examples/contributed/bsd_userspace.su ./Lib/BME280/examples/contributed/linux_userspace.cyclo ./Lib/BME280/examples/contributed/linux_userspace.d ./Lib/BME280/examples/contributed/linux_userspace.o ./Lib/BME280/examples/contributed/linux_userspace.su

.PHONY: clean-Lib-2f-BME280-2f-examples-2f-contributed

