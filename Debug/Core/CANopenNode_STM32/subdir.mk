################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/CANopenNode_STM32/CO_app_STM32.c \
../Core/CANopenNode_STM32/CO_driver_STM32.c \
../Core/CANopenNode_STM32/CO_storageBlank.c \
../Core/CANopenNode_STM32/OD.c 

OBJS += \
./Core/CANopenNode_STM32/CO_app_STM32.o \
./Core/CANopenNode_STM32/CO_driver_STM32.o \
./Core/CANopenNode_STM32/CO_storageBlank.o \
./Core/CANopenNode_STM32/OD.o 

C_DEPS += \
./Core/CANopenNode_STM32/CO_app_STM32.d \
./Core/CANopenNode_STM32/CO_driver_STM32.d \
./Core/CANopenNode_STM32/CO_storageBlank.d \
./Core/CANopenNode_STM32/OD.d 


# Each subdirectory must supply rules for building sources it contributes
Core/CANopenNode_STM32/%.o Core/CANopenNode_STM32/%.su Core/CANopenNode_STM32/%.cyclo: ../Core/CANopenNode_STM32/%.c Core/CANopenNode_STM32/subdir.mk
	arm-none-eabi-gcc -gdwarf-4 "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../CANopenNode -I"../Core/CANopenNode_STM32" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-CANopenNode_STM32

clean-Core-2f-CANopenNode_STM32:
	-$(RM) ./Core/CANopenNode_STM32/CO_app_STM32.cyclo ./Core/CANopenNode_STM32/CO_app_STM32.d ./Core/CANopenNode_STM32/CO_app_STM32.o ./Core/CANopenNode_STM32/CO_app_STM32.su ./Core/CANopenNode_STM32/CO_driver_STM32.cyclo ./Core/CANopenNode_STM32/CO_driver_STM32.d ./Core/CANopenNode_STM32/CO_driver_STM32.o ./Core/CANopenNode_STM32/CO_driver_STM32.su ./Core/CANopenNode_STM32/CO_storageBlank.cyclo ./Core/CANopenNode_STM32/CO_storageBlank.d ./Core/CANopenNode_STM32/CO_storageBlank.o ./Core/CANopenNode_STM32/CO_storageBlank.su ./Core/CANopenNode_STM32/OD.cyclo ./Core/CANopenNode_STM32/OD.d ./Core/CANopenNode_STM32/OD.o ./Core/CANopenNode_STM32/OD.su

.PHONY: clean-Core-2f-CANopenNode_STM32

