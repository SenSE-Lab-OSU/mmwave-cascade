@echo off
:: ============================================================
:: build_cascade_mss.bat
:: Builds the 2-chip cascade raw capture MSS application
:: Output: am273x\am273x_cascade_mss.xer5f
:: ============================================================

:: Step 1 - Set up SDK environment
cd /d C:\ti\mmwave_mcuplus_sdk_04_04_00_01\mmwave_mcuplus_sdk_04_04_00_01\scripts\windows
call setenv.bat

:: Step 2 - Override CCS paths (installed as ccs1240, setenv.bat expects ccs1220)
set CCS_INSTALL_PATH=C:/ti/ccs1240
set R5F_CLANG_INSTALL_PATH=C:/ti/ccs1240/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS
set C66X_CODEGEN_INSTALL_PATH=C:/ti/ccs1240/ccs/tools/compiler/ti-cgt-c6000_8.3.12
set XDC_INSTALL_PATH=C:/ti/ccs1240/xdctools_3_62_01_16_core

:: Step 3 - Add gmake and cygwin to PATH
set PATH=C:\ti\ccs1240\ccs\utils\bin;C:\ti\ccs1240\ccs\utils\cygwin;%PATH%

:: Step 4 - Build
cd /d C:\ti\mmwave_mcuplus_sdk_04_04_00_01\mmwave_mcuplus_sdk_04_04_00_01\ti\utils\test\cascade
gmake -f makefile test

echo.
echo ============================================================
echo Build complete.
echo Output: am273x\am273x_cascade_mss.xer5f
echo Load this file onto the R5F core via CCS.
echo ============================================================
pause
