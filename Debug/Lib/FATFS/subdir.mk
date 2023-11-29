################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/FATFS/ff.c \
../Lib/FATFS/ffsystem.c \
../Lib/FATFS/ffunicode.c 

OBJS += \
./Lib/FATFS/ff.o \
./Lib/FATFS/ffsystem.o \
./Lib/FATFS/ffunicode.o 

C_DEPS += \
./Lib/FATFS/ff.d \
./Lib/FATFS/ffsystem.d \
./Lib/FATFS/ffunicode.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/FATFS/%.o Lib/FATFS/%.su Lib/FATFS/%.cyclo: ../Lib/FATFS/%.c Lib/FATFS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -I../Lib -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-FATFS

clean-Lib-2f-FATFS:
	-$(RM) ./Lib/FATFS/ff.cyclo ./Lib/FATFS/ff.d ./Lib/FATFS/ff.o ./Lib/FATFS/ff.su ./Lib/FATFS/ffsystem.cyclo ./Lib/FATFS/ffsystem.d ./Lib/FATFS/ffsystem.o ./Lib/FATFS/ffsystem.su ./Lib/FATFS/ffunicode.cyclo ./Lib/FATFS/ffunicode.d ./Lib/FATFS/ffunicode.o ./Lib/FATFS/ffunicode.su

.PHONY: clean-Lib-2f-FATFS

