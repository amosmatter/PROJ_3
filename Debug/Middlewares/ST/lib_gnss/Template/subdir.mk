################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/lib_gnss/Template/gnss_lib_config.c 

OBJS += \
./Middlewares/ST/lib_gnss/Template/gnss_lib_config.o 

C_DEPS += \
./Middlewares/ST/lib_gnss/Template/gnss_lib_config.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/lib_gnss/Template/%.o Middlewares/ST/lib_gnss/Template/%.su Middlewares/ST/lib_gnss/Template/%.cyclo: ../Middlewares/ST/lib_gnss/Template/%.c Middlewares/ST/lib_gnss/Template/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -I../Lib -I../lib_gnss/Target -I../Middlewares/ST/lib_gnss/LibGNSS/Inc -I../Middlewares/ST/lib_gnss/LibNMEA/Inc -I../Middlewares/ST/lib_gnss/Template -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-lib_gnss-2f-Template

clean-Middlewares-2f-ST-2f-lib_gnss-2f-Template:
	-$(RM) ./Middlewares/ST/lib_gnss/Template/gnss_lib_config.cyclo ./Middlewares/ST/lib_gnss/Template/gnss_lib_config.d ./Middlewares/ST/lib_gnss/Template/gnss_lib_config.o ./Middlewares/ST/lib_gnss/Template/gnss_lib_config.su

.PHONY: clean-Middlewares-2f-ST-2f-lib_gnss-2f-Template

