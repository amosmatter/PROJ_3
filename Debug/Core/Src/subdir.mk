################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/IMU_DMP_init.c \
../Core/Src/app_freertos.c \
../Core/Src/delay.c \
../Core/Src/main.c \
../Core/Src/spi_common.c \
../Core/Src/spi_diskio.c \
../Core/Src/stm32u5xx_hal_msp.c \
../Core/Src/stm32u5xx_hal_timebase_tim.c \
../Core/Src/stm32u5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32u5xx.c \
../Core/Src/task_GPS.c \
../Core/Src/task_IMU.c \
../Core/Src/task_PTH.c \
../Core/Src/task_SD.c 

OBJS += \
./Core/Src/IMU_DMP_init.o \
./Core/Src/app_freertos.o \
./Core/Src/delay.o \
./Core/Src/main.o \
./Core/Src/spi_common.o \
./Core/Src/spi_diskio.o \
./Core/Src/stm32u5xx_hal_msp.o \
./Core/Src/stm32u5xx_hal_timebase_tim.o \
./Core/Src/stm32u5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32u5xx.o \
./Core/Src/task_GPS.o \
./Core/Src/task_IMU.o \
./Core/Src/task_PTH.o \
./Core/Src/task_SD.o 

C_DEPS += \
./Core/Src/IMU_DMP_init.d \
./Core/Src/app_freertos.d \
./Core/Src/delay.d \
./Core/Src/main.d \
./Core/Src/spi_common.d \
./Core/Src/spi_diskio.d \
./Core/Src/stm32u5xx_hal_msp.d \
./Core/Src/stm32u5xx_hal_timebase_tim.d \
./Core/Src/stm32u5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32u5xx.d \
./Core/Src/task_GPS.d \
./Core/Src/task_IMU.d \
./Core/Src/task_PTH.d \
./Core/Src/task_SD.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -I../Lib -I../Core/ThreadSafe -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/IMU_DMP_init.cyclo ./Core/Src/IMU_DMP_init.d ./Core/Src/IMU_DMP_init.o ./Core/Src/IMU_DMP_init.su ./Core/Src/app_freertos.cyclo ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/delay.cyclo ./Core/Src/delay.d ./Core/Src/delay.o ./Core/Src/delay.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/spi_common.cyclo ./Core/Src/spi_common.d ./Core/Src/spi_common.o ./Core/Src/spi_common.su ./Core/Src/spi_diskio.cyclo ./Core/Src/spi_diskio.d ./Core/Src/spi_diskio.o ./Core/Src/spi_diskio.su ./Core/Src/stm32u5xx_hal_msp.cyclo ./Core/Src/stm32u5xx_hal_msp.d ./Core/Src/stm32u5xx_hal_msp.o ./Core/Src/stm32u5xx_hal_msp.su ./Core/Src/stm32u5xx_hal_timebase_tim.cyclo ./Core/Src/stm32u5xx_hal_timebase_tim.d ./Core/Src/stm32u5xx_hal_timebase_tim.o ./Core/Src/stm32u5xx_hal_timebase_tim.su ./Core/Src/stm32u5xx_it.cyclo ./Core/Src/stm32u5xx_it.d ./Core/Src/stm32u5xx_it.o ./Core/Src/stm32u5xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32u5xx.cyclo ./Core/Src/system_stm32u5xx.d ./Core/Src/system_stm32u5xx.o ./Core/Src/system_stm32u5xx.su ./Core/Src/task_GPS.cyclo ./Core/Src/task_GPS.d ./Core/Src/task_GPS.o ./Core/Src/task_GPS.su ./Core/Src/task_IMU.cyclo ./Core/Src/task_IMU.d ./Core/Src/task_IMU.o ./Core/Src/task_IMU.su ./Core/Src/task_PTH.cyclo ./Core/Src/task_PTH.d ./Core/Src/task_PTH.o ./Core/Src/task_PTH.su ./Core/Src/task_SD.cyclo ./Core/Src/task_SD.d ./Core/Src/task_SD.o ./Core/Src/task_SD.su

.PHONY: clean-Core-2f-Src

