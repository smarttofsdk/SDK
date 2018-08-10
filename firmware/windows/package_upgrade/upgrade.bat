@echo off
rem get old calibration data by open device

echo there are total 3 steps to upgrade,make sure not close the cmd before all the 3 steps finished
echo step 1

call dmcam-cli_static143.exe -e version 

rem construct new calibration data struction
call bin_data_upgrade.exe 1

rem upgrade new firmware
echo step 2
call dmcam-cli_static143.exe -f TM-E2_HW20_SW150_20180809.bin

rem wait mcu reset
echo do not close 
ping /n 10 127.0.0.1 >nul


call bin_data_upgrade.exe 6

echo do not close
ping /n 15 127.0.0.1 >nul

rem download new calibration data to module
echo step 3
call bin_data_upgrade.exe 2

echo now upgrade finished
pause