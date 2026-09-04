@echo off
:: ============================================================
:: build_cascade_mss_ddma.bat
:: Builds the DDMA variant in-place from am273xDDMA\
:: Output: am273xDDMA\am273xDDMA_cascade_mss.xer5f
:: ============================================================

cd /d C:\ti\mmwave_mcuplus_sdk_04_04_00_01\mmwave_mcuplus_sdk_04_04_00_01\scripts\windows
call setenv.bat

set CCS_INSTALL_PATH=C:/ti/ccs1200
set R5F_CLANG_INSTALL_PATH=C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-armllvm_2.1.0.LTS
set C66X_CODEGEN_INSTALL_PATH=C:/ti/ccs1200/ccs/tools/compiler/ti-cgt-c6000_8.3.12
set XDC_INSTALL_PATH=C:/ti/ccs1200/xdctools_3_62_01_16_core

set PATH=C:\ti\ccs1200\ccs\utils\bin;C:\ti\ccs1200\ccs\utils\cygwin;%PATH%

cd /d C:\ti\mmwave_mcuplus_sdk_04_04_00_01\mmwave_mcuplus_sdk_04_04_00_01\ti\utils\test\cascade
:: testClean is mandatory: the object dir is shared with the TDMA build
C:\ti\ccs1200\ccs\utils\bin\gmake -f makefile_ddma testClean test

echo.
echo ============================================================
echo Build complete.
echo DDMA binary: am273xDDMA\am273xDDMA_cascade_mss.xer5f
echo ============================================================
pause
