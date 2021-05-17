################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sim7600_driver/src/LL_GetTick.c \
../sim7600_driver/src/sim7600_uart_dma.c 

OBJS += \
./sim7600_driver/src/LL_GetTick.o \
./sim7600_driver/src/sim7600_uart_dma.o 

C_DEPS += \
./sim7600_driver/src/LL_GetTick.d \
./sim7600_driver/src/sim7600_uart_dma.d 


# Each subdirectory must supply rules for building sources it contributes
sim7600_driver/src/LL_GetTick.o: ../sim7600_driver/src/LL_GetTick.c sim7600_driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Bom/STM32CubeIDE/workspace_1.6.1/sim7600LL/sim7600_driver/include" -I"C:/Users/Bom/STM32CubeIDE/workspace_1.6.1/sim7600LL/vs1003/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"sim7600_driver/src/LL_GetTick.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
sim7600_driver/src/sim7600_uart_dma.o: ../sim7600_driver/src/sim7600_uart_dma.c sim7600_driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Bom/STM32CubeIDE/workspace_1.6.1/sim7600LL/sim7600_driver/include" -I"C:/Users/Bom/STM32CubeIDE/workspace_1.6.1/sim7600LL/vs1003/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"sim7600_driver/src/sim7600_uart_dma.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

