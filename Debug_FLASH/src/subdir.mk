################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/SEGGER_RTT.c \
../src/SEGGER_RTT_Syscalls_GCC.c \
../src/SEGGER_RTT_printf.c \
../src/main.c 

S_UPPER_SRCS += \
C:/Users/Admin/Downloads/RTT/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./src/SEGGER_RTT.o \
./src/SEGGER_RTT_ASM_ARMv7M.o \
./src/SEGGER_RTT_Syscalls_GCC.o \
./src/SEGGER_RTT_printf.o \
./src/main.o 

C_DEPS += \
./src/SEGGER_RTT.d \
./src/SEGGER_RTT_Syscalls_GCC.d \
./src/SEGGER_RTT_printf.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C Compiler'
	arm-none-eabi-gcc "@src/SEGGER_RTT.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/SEGGER_RTT_ASM_ARMv7M.o: C:/Users/Admin/Downloads/RTT/SEGGER_RTT_ASM_ARMv7M.S
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS Assembler'
	arm-none-eabi-gcc "@src/SEGGER_RTT_ASM_ARMv7M.args" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


