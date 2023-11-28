################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/ICM_20948/src/util/ICM_20948_C.c 

OBJS += \
./Lib/ICM_20948/src/util/ICM_20948_C.o 

C_DEPS += \
./Lib/ICM_20948/src/util/ICM_20948_C.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/ICM_20948/src/util/%.o Lib/ICM_20948/src/util/%.su Lib/ICM_20948/src/util/%.cyclo: ../Lib/ICM_20948/src/util/%.c Lib/ICM_20948/src/util/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -I../Lib -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-ICM_20948-2f-src-2f-util

clean-Lib-2f-ICM_20948-2f-src-2f-util:
	-$(RM) ./Lib/ICM_20948/src/util/ICM_20948_C.cyclo ./Lib/ICM_20948/src/util/ICM_20948_C.d ./Lib/ICM_20948/src/util/ICM_20948_C.o ./Lib/ICM_20948/src/util/ICM_20948_C.su

.PHONY: clean-Lib-2f-ICM_20948-2f-src-2f-util

