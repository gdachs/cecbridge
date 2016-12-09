################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/cecbridge.c \
../Src/stm32f0xx_hal_msp.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/cecbridge.o \
./Src/stm32f0xx_hal_msp.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/cecbridge.d \
./Src/stm32f0xx_hal_msp.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F042x6 -I"/home/gda/stm32workspace/cecbridge/Inc" -I"/home/gda/stm32workspace/cecbridge/Drivers/STM32F0xx_HAL_Driver/Inc" -I"/home/gda/stm32workspace/cecbridge/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"/home/gda/stm32workspace/cecbridge/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/gda/stm32workspace/cecbridge/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/gda/stm32workspace/cecbridge/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"/home/gda/stm32workspace/cecbridge/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


