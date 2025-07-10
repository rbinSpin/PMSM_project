################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../28335_RAM_lnk.cmd \
C:/ti/controlSUITE/device_support/f2833x/v142/DSP2833x_headers/cmd/DSP2833x_Headers_nonBIOS.cmd 

LIB_SRCS += \
C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/IQmath/c28/lib/IQmath.lib \
C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/IQmath/c28/lib/IQmath_fpu32.lib 

ASM_SRCS += \
../DLOG4CHC.asm \
../DSP2833x_ADC_cal.asm \
C:/ti/controlSUITE/device_support/f2833x/v142/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm \
C:/ti/controlSUITE/device_support/f2833x/v142/DSP2833x_common/source/DSP2833x_usDelay.asm 

C_SRCS += \
C:/ti/myprojects/headers/source/DSP2833x_GlobalVariableDefs.c \
../HVPM_Sensored-DevInit_F2833x.c \
../main.c 

C_DEPS += \
./DSP2833x_GlobalVariableDefs.d \
./HVPM_Sensored-DevInit_F2833x.d \
./main.d 

OBJS += \
./DLOG4CHC.obj \
./DSP2833x_ADC_cal.obj \
./DSP2833x_CodeStartBranch.obj \
./DSP2833x_GlobalVariableDefs.obj \
./DSP2833x_usDelay.obj \
./HVPM_Sensored-DevInit_F2833x.obj \
./main.obj 

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
"main.obj" 

C_DEPS__QUOTED += \
"DSP2833x_GlobalVariableDefs.d" \
"HVPM_Sensored-DevInit_F2833x.d" \
"main.d" 

ASM_DEPS__QUOTED += \
"DLOG4CHC.d" \
"DSP2833x_ADC_cal.d" \
"DSP2833x_CodeStartBranch.d" \
"DSP2833x_usDelay.d" 

ASM_SRCS__QUOTED += \
"../DLOG4CHC.asm" \
"../DSP2833x_ADC_cal.asm" \
"C:/ti/controlSUITE/device_support/f2833x/v142/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm" \
"C:/ti/controlSUITE/device_support/f2833x/v142/DSP2833x_common/source/DSP2833x_usDelay.asm" 

C_SRCS__QUOTED += \
"C:/ti/myprojects/headers/source/DSP2833x_GlobalVariableDefs.c" \
"../HVPM_Sensored-DevInit_F2833x.c" 


