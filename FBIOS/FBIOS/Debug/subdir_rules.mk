################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
FBIOScfg.cmd: ../FBIOS.tcf
	@echo 'Building file: $<'
	@echo 'Invoking: TConf'
	"C:/ti/bios_5_42_01_09/xdctools/tconf" -b -Dconfig.importPath="C:/ti/bios_5_42_01_09/packages;" "$<"
	@echo 'Finished building: $<'
	@echo ' '

FBIOScfg.s??: | FBIOScfg.cmd
FBIOScfg_c.c: | FBIOScfg.cmd
FBIOScfg.h: | FBIOScfg.cmd
FBIOScfg.h??: | FBIOScfg.cmd
FBIOS.cdb: | FBIOScfg.cmd

FBIOScfg.obj: ./FBIOScfg.s?? $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c6000_7.4.7/bin/cl6x" -mv6700 --abi=coffabi -g --include_path="C:/ti/ccsv6/tools/compiler/c6000_7.4.7/include" --include_path="C:/Program Files/C6xCSL/include" --include_path="D:/Em/DSK6713/c6000/dsk6713/include" --include_path="D:/Em/FBIOS/Debug" --include_path="C:/ti/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/ti/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=c6713 --define=CHIP_6713 --diag_wrap=off --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="FBIOScfg.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

FBIOScfg_c.obj: ./FBIOScfg_c.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c6000_7.4.7/bin/cl6x" -mv6700 --abi=coffabi -g --include_path="C:/ti/ccsv6/tools/compiler/c6000_7.4.7/include" --include_path="C:/Program Files/C6xCSL/include" --include_path="D:/Em/DSK6713/c6000/dsk6713/include" --include_path="D:/Em/FBIOS/Debug" --include_path="C:/ti/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/ti/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=c6713 --define=CHIP_6713 --diag_wrap=off --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="FBIOScfg_c.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

main.obj: ../main.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/ti/ccsv6/tools/compiler/c6000_7.4.7/bin/cl6x" -mv6700 --abi=coffabi -g --include_path="C:/ti/ccsv6/tools/compiler/c6000_7.4.7/include" --include_path="C:/Program Files/C6xCSL/include" --include_path="D:/Em/DSK6713/c6000/dsk6713/include" --include_path="D:/Em/FBIOS/Debug" --include_path="C:/ti/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/ti/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=c6713 --define=CHIP_6713 --diag_wrap=off --diag_warning=225 --display_error_number --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


