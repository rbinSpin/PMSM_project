################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../28335_RAM_lnk.cmd \
../DSP2833x_Headers_nonBIOS.cmd 

LIB_SRCS += \
../IQmath_fpu32.lib \
../rts2800_fpu32_fast_supplement.lib 

ASM_SRCS += \
../DLOG4CHC.asm \
../DSP2833x_ADC_cal.asm \
../DSP2833x_CodeStartBranch.asm \
../DSP2833x_usDelay.asm 

C_SRCS += \
../DSP2833x_GlobalVariableDefs.c \
../HVPM_Sensored-DevInit_F2833x.c \
../main.c \
../spi_transfer.c 

C_DEPS += \
./DSP2833x_GlobalVariableDefs.d \
./HVPM_Sensored-DevInit_F2833x.d \
./main.d \
./spi_transfer.d 

OBJS += \
./DLOG4CHC.obj \
./DSP2833x_ADC_cal.obj \
./DSP2833x_CodeStartBranch.obj \
./DSP2833x_GlobalVariableDefs.obj \
./DSP2833x_usDelay.obj \
./HVPM_Sensored-DevInit_F2833x.obj \
./main.obj \
./spi_transfer.obj 

ASM_DEPS += \
./DLOG4CHC.d \
./DSP2833x_ADC_cal.d \
./DSP2833x_CodeStartBranch.d \
./DSP2833x_usDelay.d 

OBJS__QUOTED += \
"DLOG4CHC.obj" \
"DSP2833x_ADC_cal.obj" \
"DSP2833x_CodeStartBranch.obj" \
"DSP2833x_GlobalVariableDefs.obj" \
"DSP2833x_usDelay.obj" \
"HVPM_Sensored-DevInit_F2833x.obj" \
"main.obj" \
"spi_transfer.obj" 

C_DEPS__QUOTED += \
"DSP2833x_GlobalVariableDefs.d" \
"HVPM_Sensored-DevInit_F2833x.d" \
"main.d" \
"spi_transfer.d" 

ASM_DEPS__QUOTED += \
"DLOG4CHC.d" \
"DSP2833x_ADC_cal.d" \
"DSP2833x_CodeStartBranch.d" \
"DSP2833x_usDelay.d" 

ASM_SRCS__QUOTED += \
"../DLOG4CHC.asm" \
"../DSP2833x_ADC_cal.asm" \
"../DSP2833x_CodeStartBranch.asm" \
"../DSP2833x_usDelay.asm" 

C_SRCS__QUOTED += \
"../DSP2833x_GlobalVariableDefs.c" \
"../HVPM_Sensored-DevInit_F2833x.c" \
"../spi_transfer.c" 


