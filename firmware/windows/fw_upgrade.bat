@echo off

REM set upgradebin= TM-E2_HW20_SW143_20180724.bin

call dmcam-cli_static143.exe -v debug -e "print info" >moduleinfo.txt
findstr /C:"Product" moduleinfo.txt >Product.txt
for /f "tokens=2 delims=:" %%i in (Product.txt) do set product=%%i

echo now Product:%Product%

if %Product% == TM-E2-1.0 goto E2
if %Product% == TM-E3-1.0 goto E3
if %Product% == TM-E4-1.0 goto E4

:E2
echo now upgrade for E2
call dmcam-cli.exe -f TC-General-1.0_HW30*.bin
goto end

:E3
echo now upgrade for E3
call dmcam-cli.exe -f TC-General-2.0*.bin
goto end

:E4
echo now upgrade for E4
call dmcam-cli.exe -f TC-General-1.0*.bin
goto end

:end
pause >nul
