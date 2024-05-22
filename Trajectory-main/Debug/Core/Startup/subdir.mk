################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/BasicMathFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/BayesFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/CommonTables" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/ComplexMathFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/ControllerFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/DistanceFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/FastMathFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/FilteringFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/InterpolationFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/MatrixFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/QuaternionMathFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/StatisticsFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/SupportFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/SVMFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/TransformFunctions" -I"/Users/firm/STM32CubeIDE/workspace_1.15.1/Trajectory/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

