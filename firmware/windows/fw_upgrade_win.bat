@echo off

set upgradebin= TM-E2_HW20_SW143_20180724.bin
REM set upgradebinHW= TM-E2_HW100_SW132_20180605.bin

call dmcam-cli-140.exe -v debug -e "print info" >moduleinfo.txt

findstr /C:"MCU Firmware Version" moduleinfo.txt >version.txt

for /f "tokens=2 delims=:" %%i in (version.txt) do set ver=%%i

echo now target:%ver%

if %ver% LEQ 131 goto cli131
if %ver% GTR 131 goto cli140

:cli131
	echo use in cli131
	call dmcam-cli-131.exe -f %upgradebin% 
	goto end
	
:cli140
	echo use in cli140
	call dmcam-cli-140.exe -f %upgradebin%
	goto end

:end
	pause >nul
