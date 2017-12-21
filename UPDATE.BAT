@echo OFF

IF %1.==. GOTO help

set FILE=%1
set pInstallDir=NULL

set KEY_NAME=HKEY_LOCAL_MACHINE\SOFTWARE\Analog Devices\PlutoSDR-M2k-USB-Win-Drivers\Settings
FOR /F "tokens=2*" %%A IN ('REG.exe query "%KEY_NAME%" /v "InstallPath"') DO (set pInstallDir=%%B)

IF "%pInstallDir%"=="NULL" (
	echo PlutoSDR-M2k-USB-Win-Drivers not installed && exit /b 1
)

REM "%pInstallDir%\dfu-util.exe" -l || exit /b 1

for /F %%i in ("%FILE%") do @set NAME=%%~nxi

if %NAME%==pluto.dfu goto firmware
if %NAME%==uboot-env.dfu goto ubootenv
goto help

:firmware
"%pInstallDir%\dfu-util.exe" -d 0456:b673,0456:b674 -D %FILE% -a firmware.dfu
exit /b 0

:ubootenv
"%pInstallDir%\dfu-util.exe" -d 0456:b673,0456:b674 -D %FILE% -a uboot-env.dfu
exit /b 0

:help
echo PlutoSDR DFU update utility
echo Usage: %0 PATH-TO\pluto.dfu
exit /b 0
