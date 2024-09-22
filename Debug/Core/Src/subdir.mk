################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ax25_generator.c \
../Core/Src/com_debug.c \
../Core/Src/dma.c \
../Core/Src/error_handler.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/obc_interface.c \
../Core/Src/stm32_adv_trace_if.c \
../Core/Src/stm32_lpm_if.c \
../Core/Src/stm32wlxx_hal_msp.c \
../Core/Src/stm32wlxx_it.c \
../Core/Src/subghz.c \
../Core/Src/sys_app.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32wlxx.c \
../Core/Src/tim.c \
../Core/Src/timer_if.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/ax25_generator.o \
./Core/Src/com_debug.o \
./Core/Src/dma.o \
./Core/Src/error_handler.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/obc_interface.o \
./Core/Src/stm32_adv_trace_if.o \
./Core/Src/stm32_lpm_if.o \
./Core/Src/stm32wlxx_hal_msp.o \
./Core/Src/stm32wlxx_it.o \
./Core/Src/subghz.o \
./Core/Src/sys_app.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32wlxx.o \
./Core/Src/tim.o \
./Core/Src/timer_if.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/ax25_generator.d \
./Core/Src/com_debug.d \
./Core/Src/dma.d \
./Core/Src/error_handler.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/obc_interface.d \
./Core/Src/stm32_adv_trace_if.d \
./Core/Src/stm32_lpm_if.d \
./Core/Src/stm32wlxx_hal_msp.d \
./Core/Src/stm32wlxx_it.d \
./Core/Src/subghz.d \
./Core/Src/sys_app.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32wlxx.d \
./Core/Src/tim.d \
./Core/Src/timer_if.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -I../SubGHz_Phy/App -I../SubGHz_Phy/Target -I../Utilities/trace/adv_trace -I../Utilities/misc -I../Utilities/sequencer -I../Utilities/timer -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I"/Users/sajanduwal/STM32CubeIDE/workspace_1.13.1_v2/Satellite_COM/Drivers/STM32WLxx_Nucleo" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ax25_generator.cyclo ./Core/Src/ax25_generator.d ./Core/Src/ax25_generator.o ./Core/Src/ax25_generator.su ./Core/Src/com_debug.cyclo ./Core/Src/com_debug.d ./Core/Src/com_debug.o ./Core/Src/com_debug.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/error_handler.cyclo ./Core/Src/error_handler.d ./Core/Src/error_handler.o ./Core/Src/error_handler.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/obc_interface.cyclo ./Core/Src/obc_interface.d ./Core/Src/obc_interface.o ./Core/Src/obc_interface.su ./Core/Src/stm32_adv_trace_if.cyclo ./Core/Src/stm32_adv_trace_if.d ./Core/Src/stm32_adv_trace_if.o ./Core/Src/stm32_adv_trace_if.su ./Core/Src/stm32_lpm_if.cyclo ./Core/Src/stm32_lpm_if.d ./Core/Src/stm32_lpm_if.o ./Core/Src/stm32_lpm_if.su ./Core/Src/stm32wlxx_hal_msp.cyclo ./Core/Src/stm32wlxx_hal_msp.d ./Core/Src/stm32wlxx_hal_msp.o ./Core/Src/stm32wlxx_hal_msp.su ./Core/Src/stm32wlxx_it.cyclo ./Core/Src/stm32wlxx_it.d ./Core/Src/stm32wlxx_it.o ./Core/Src/stm32wlxx_it.su ./Core/Src/subghz.cyclo ./Core/Src/subghz.d ./Core/Src/subghz.o ./Core/Src/subghz.su ./Core/Src/sys_app.cyclo ./Core/Src/sys_app.d ./Core/Src/sys_app.o ./Core/Src/sys_app.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32wlxx.cyclo ./Core/Src/system_stm32wlxx.d ./Core/Src/system_stm32wlxx.o ./Core/Src/system_stm32wlxx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/timer_if.cyclo ./Core/Src/timer_if.d ./Core/Src/timer_if.o ./Core/Src/timer_if.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

