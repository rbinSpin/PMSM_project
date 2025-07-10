################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --opt_for_speed=0 --include_path="C:/ti/myprojects/common/include" --include_path="C:/ti/myprojects/headers/include" --include_path="C:/ti/myprojects/common/source" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/FPUfastRTS/c28/lib" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

DSP2833x_CodeStartBranch.obj: C:/ti/controlSUITE/device_support/f2833x/v142/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --opt_for_speed=0 --include_path="C:/ti/myprojects/common/include" --include_path="C:/ti/myprojects/headers/include" --include_path="C:/ti/myprojects/common/source" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/FPUfastRTS/c28/lib" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

DSP2833x_GlobalVariableDefs.obj: C:/ti/myprojects/headers/source/DSP2833x_GlobalVariableDefs.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --opt_for_speed=0 --include_path="C:/ti/myprojects/common/include" --include_path="C:/ti/myprojects/headers/include" --include_path="C:/ti/myprojects/common/source" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/FPUfastRTS/c28/lib" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

DSP2833x_usDelay.obj: C:/ti/controlSUITE/device_support/f2833x/v142/DSP2833x_common/source/DSP2833x_usDelay.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --opt_for_speed=0 --include_path="C:/ti/myprojects/common/include" --include_path="C:/ti/myprojects/headers/include" --include_path="C:/ti/myprojects/common/source" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/FPUfastRTS/c28/lib" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --opt_for_speed=0 --include_path="C:/ti/myprojects/common/include" --include_path="C:/ti/myprojects/headers/include" --include_path="C:/ti/myprojects/common/source" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/FPUfastRTS/c28/lib" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

main.obj: ../main.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --opt_for_speed=0 --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --include_path="C:/ti/myprojects/examples/PMSM_28335" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="C:/ti/myprojects/common/include" --include_path="C:/ti/myprojects/headers/include" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/FPUfastRTS/c28/lib" --include_path="C:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --include_path="C:/ti/myprojects/examples/PMSM_28335" --include_path="C:/ti/myprojects/common/include" --include_path="C:/ti/myprojects/headers/include" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/IQmath/c28/include" --include_path="C:/ti/c2000/C2000Ware_4_01_00_00/libraries/math/FPUfastRTS/c28/lib" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.3" --include_path="C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --include_path="C:/ti/myprojects/common/source" --include_path="C:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


