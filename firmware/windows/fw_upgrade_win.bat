@echo off

REM set upgradebin= TM-E2_HW20_SW143_20180724.bin

call dmcam-cli_static143.exe -v debug -e "print info" >moduleinfo.txt

findstr /C:"MCU Firmware Version" moduleinfo.txt >version.txt
findstr /C:"Its Hardware Version" moduleinfo.txt >hardversion.txt
findstr /C:"TFC Firmware Version" moduleinfo.txt >tfcversion.txt


for /f "tokens=2 delims=:" %%i in (version.txt) do set ver=%%i
for /f "tokens=2 delims=:" %%i in (hardversion.txt) do set hardver=%%i
for /f "tokens=2 delims=:" %%i in (tfcversion.txt) do set tfcver=%%i

echo now target:%ver%
echo now hardver:%hardver%
echo now tfcver:%tfcver%

if %ver% LEQ 131 goto before132

if %tfcver% GTR 0 if %tfcver% LEQ 145 goto forbidden

if %hardver% EQU 20 if %ver% LEQ 143 goto package_upgrade
if %hardver% EQU 20 if %ver% GTR 143 goto nopack_HW20
if %hardver% EQU 30 goto nopack_HW30

:package_upgrade
	echo use in package_upgrade
	cd .\package_upgrade
	call upgrade.bat
	
	pause
	goto end

:nopack_HW20
	echo use in nopack_HW20
	cd .\nopack_HW20
	call upgrade.bat
	
	pause
	goto end

:nopack_HW30
	echo use in nopack_HW30
	cd .\nopack_HW30
	call upgrade.bat
	
	pause
	goto end
	
:forbidden
	echo your version is not correct,please contact the FAE for support
	pause
	goto end
	
	
:before132
	echo your version is too old,please first upgrade to the version 143,
	echo you can goto the uploaddir to upgrade to 143,and then do the next upgrade
	
	pause
	goto end

:end
	pause >nul
