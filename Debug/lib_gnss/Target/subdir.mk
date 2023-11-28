################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib_gnss/Target/gnss_lib_config.c 

OBJS += \
./lib_gnss/Target/gnss_lib_config.o 

C_DEPS += \
./lib_gnss/Target/gnss_lib_config.d 


# Each subdirectory must supply rules for building sources it contributes
lib_gnss/Target/%.o lib_gnss/Target/%.su lib_gnss/Target/%.cyclo: ../lib_gnss/Target/%.c lib_gnss/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -I../Lib -I../lib_gnss/Target -I../Middlewares/ST/lib_gnss/LibGNSS/Inc -I../Middlewares/ST/lib_gnss/LibNMEA/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib_gnss-2f-Target

clean-lib_gnss-2f-Target:
	-$(RM) ./lib_gnss/Target/gnss_lib_config.cyclo ./lib_gnss/Target/gnss_lib_config.d ./lib_gnss/Target/gnss_lib_config.o ./lib_gnss/Target/gnss_lib_config.su

.PHONY: clean-lib_gnss-2f-Target

