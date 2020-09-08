################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CRC16.c \
../Core/Src/ami.c \
../Core/Src/ami_protocol.c \
../Core/Src/flash_stm32f4.c \
../Core/Src/hal_func_redef.c \
../Core/Src/m95m01_eeprom.c \
../Core/Src/main.c \
../Core/Src/memory.c \
../Core/Src/memory_status.c \
../Core/Src/mt29fxg08_nand.c \
../Core/Src/nand_block_table.c \
../Core/Src/nand_ecc.c \
../Core/Src/post_module.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/CRC16.o \
./Core/Src/ami.o \
./Core/Src/ami_protocol.o \
./Core/Src/flash_stm32f4.o \
./Core/Src/hal_func_redef.o \
./Core/Src/m95m01_eeprom.o \
./Core/Src/main.o \
./Core/Src/memory.o \
./Core/Src/memory_status.o \
./Core/Src/mt29fxg08_nand.o \
./Core/Src/nand_block_table.o \
./Core/Src/nand_ecc.o \
./Core/Src/post_module.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/CRC16.d \
./Core/Src/ami.d \
./Core/Src/ami_protocol.d \
./Core/Src/flash_stm32f4.d \
./Core/Src/hal_func_redef.d \
./Core/Src/m95m01_eeprom.d \
./Core/Src/main.d \
./Core/Src/memory.d \
./Core/Src/memory_status.d \
./Core/Src/mt29fxg08_nand.d \
./Core/Src/nand_block_table.d \
./Core/Src/nand_ecc.d \
./Core/Src/post_module.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c11 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F427xx -I"D:/diplom/Bootloader/Core/Inc" -I"D:/diplom/Bootloader/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/diplom/Bootloader/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/diplom/Bootloader/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/diplom/Bootloader/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


