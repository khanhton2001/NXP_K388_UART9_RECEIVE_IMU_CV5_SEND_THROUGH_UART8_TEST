################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include generate/src/subdir.mk
-include board/subdir.mk
-include RTD/src/subdir.mk
-include Project_Settings/Startup_Code/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_SIZE += \
TEST_CAN_S32K388_128_M7_0_0.siz \


# All Target
all: TEST_CAN_S32K388_128_M7_0_0.elf secondary-outputs

dependents:
	-cd C:/Users/Admin/workspaceS32DS.3.6.3/TEST_CAN_S32K388_128/TEST_CAN_S32K388_128_M7_1_0/Debug_FLASH && $(MAKE) all
	-cd C:/Users/Admin/workspaceS32DS.3.6.3/TEST_CAN_S32K388_128/TEST_CAN_S32K388_128_M7_2_0/Debug_FLASH && $(MAKE) all

# Tool invocations
TEST_CAN_S32K388_128_M7_0_0.elf: $(OBJS) C:/Users/Admin/workspaceS32DS.3.6.3/TEST_CAN_S32K388_128/TEST_CAN_S32K388_128_M7_0_0/Project_Settings/Linker_Files/linker_flash_c0_s32k388.ld $(USER_OBJS) C:/Users/Admin/workspaceS32DS.3.6.3/TEST_CAN_S32K388_128/TEST_CAN_S32K388_128_M7_1_0/Debug_FLASH/TEST_CAN_S32K388_128_M7_1_0.elf C:/Users/Admin/workspaceS32DS.3.6.3/TEST_CAN_S32K388_128/TEST_CAN_S32K388_128_M7_2_0/Debug_FLASH/TEST_CAN_S32K388_128_M7_2_0.elf
	@echo 'Building target: $@'
	@echo 'Invoking: Standard S32DS C Linker'
	arm-none-eabi-gcc -o "TEST_CAN_S32K388_128_M7_0_0.elf" "@TEST_CAN_S32K388_128_M7_0_0.args"  $(USER_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

TEST_CAN_S32K388_128_M7_0_0.siz: TEST_CAN_S32K388_128_M7_0_0.elf
	@echo 'Invoking: Standard S32DS Print Size'
	arm-none-eabi-size --format=berkeley TEST_CAN_S32K388_128_M7_0_0.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) ./*
	-@echo ' '

secondary-outputs: $(SECONDARY_SIZE)

.PHONY: all clean dependents
C:/Users/Admin/workspaceS32DS.3.6.3/TEST_CAN_S32K388_128/TEST_CAN_S32K388_128_M7_1_0/Debug_FLASH/TEST_CAN_S32K388_128_M7_1_0.elf:
C:/Users/Admin/workspaceS32DS.3.6.3/TEST_CAN_S32K388_128/TEST_CAN_S32K388_128_M7_2_0/Debug_FLASH/TEST_CAN_S32K388_128_M7_2_0.elf:

-include ../makefile.targets
