################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fatfs_storage.c \
../Core/Src/ff.c \
../Core/Src/freertos.c \
../Core/Src/ft5336.c \
../Core/Src/main.c \
../Core/Src/stm32746g_discovery.c \
../Core/Src/stm32746g_discovery_audio.c \
../Core/Src/stm32746g_discovery_lcd.c \
../Core/Src/stm32746g_discovery_sdram.c \
../Core/Src/stm32746g_discovery_ts.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_hal_timebase_tim.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c \
../Core/Src/wm8994.c 

OBJS += \
./Core/Src/fatfs_storage.o \
./Core/Src/ff.o \
./Core/Src/freertos.o \
./Core/Src/ft5336.o \
./Core/Src/main.o \
./Core/Src/stm32746g_discovery.o \
./Core/Src/stm32746g_discovery_audio.o \
./Core/Src/stm32746g_discovery_lcd.o \
./Core/Src/stm32746g_discovery_sdram.o \
./Core/Src/stm32746g_discovery_ts.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_hal_timebase_tim.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o \
./Core/Src/wm8994.o 

C_DEPS += \
./Core/Src/fatfs_storage.d \
./Core/Src/ff.d \
./Core/Src/freertos.d \
./Core/Src/ft5336.d \
./Core/Src/main.d \
./Core/Src/stm32746g_discovery.d \
./Core/Src/stm32746g_discovery_audio.d \
./Core/Src/stm32746g_discovery_lcd.d \
./Core/Src/stm32746g_discovery_sdram.d \
./Core/Src/stm32746g_discovery_ts.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_hal_timebase_tim.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d \
./Core/Src/wm8994.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/fatfs_storage.cyclo ./Core/Src/fatfs_storage.d ./Core/Src/fatfs_storage.o ./Core/Src/fatfs_storage.su ./Core/Src/ff.cyclo ./Core/Src/ff.d ./Core/Src/ff.o ./Core/Src/ff.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/ft5336.cyclo ./Core/Src/ft5336.d ./Core/Src/ft5336.o ./Core/Src/ft5336.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32746g_discovery.cyclo ./Core/Src/stm32746g_discovery.d ./Core/Src/stm32746g_discovery.o ./Core/Src/stm32746g_discovery.su ./Core/Src/stm32746g_discovery_audio.cyclo ./Core/Src/stm32746g_discovery_audio.d ./Core/Src/stm32746g_discovery_audio.o ./Core/Src/stm32746g_discovery_audio.su ./Core/Src/stm32746g_discovery_lcd.cyclo ./Core/Src/stm32746g_discovery_lcd.d ./Core/Src/stm32746g_discovery_lcd.o ./Core/Src/stm32746g_discovery_lcd.su ./Core/Src/stm32746g_discovery_sdram.cyclo ./Core/Src/stm32746g_discovery_sdram.d ./Core/Src/stm32746g_discovery_sdram.o ./Core/Src/stm32746g_discovery_sdram.su ./Core/Src/stm32746g_discovery_ts.cyclo ./Core/Src/stm32746g_discovery_ts.d ./Core/Src/stm32746g_discovery_ts.o ./Core/Src/stm32746g_discovery_ts.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_hal_timebase_tim.cyclo ./Core/Src/stm32f7xx_hal_timebase_tim.d ./Core/Src/stm32f7xx_hal_timebase_tim.o ./Core/Src/stm32f7xx_hal_timebase_tim.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su ./Core/Src/wm8994.cyclo ./Core/Src/wm8994.d ./Core/Src/wm8994.o ./Core/Src/wm8994.su

.PHONY: clean-Core-2f-Src

