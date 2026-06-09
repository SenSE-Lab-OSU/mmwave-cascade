@echo off
:: ============================================================
:: start_capture.bat
:: Configures DCA1000EVM FPGA and starts raw ADC data recording.
::
:: Run this BEFORE starting execution of the application in CCS.
::
:: Output file: C:\ti\capture\am273x__Raw_0.bin
::
:: Workflow:
::   1. Run this bat file
::   2. Start execution in CCS (Resume R5F core)
::   3. Wait for capture to complete
::   4. Rename C:\ti\capture\am273x__Raw_0.bin to something descriptive
::   5. Process in MATLAB
:: ============================================================

cd /d C:\ti\mmwave_studio_03_00_00_14\mmWaveStudio\PostProc

echo Configuring DCA1000EVM FPGA...
DCA1000EVM_CLI_Control.exe fpga AM273X_Capture.json

echo.
echo Starting recording...
DCA1000EVM_CLI_Control.exe start_record AM273X_Capture.json

echo.
echo ============================================================
echo Recording started.
echo Output: C:\ti\capture\am273x__Raw_0.bin
echo Now resume the R5F core in CCS to start the application.
echo When capture is done, rename the output file before next run.
echo ============================================================
pause
