@echo off
setlocal

for %%I in ("%~dp0..") do set "PROJECT_ROOT=%%~fI"
set "OPENOCD=D:\DevEnv\ti-openocd\openocd_1.5.0.75\openocd\bin\openocd.exe"
set "SCRIPTS=D:\DevEnv\ti-openocd\openocd_1.5.0.75\openocd\share\openocd\scripts"
set "CONFIG=%PROJECT_ROOT%\openocd\mspm0g3519_daplink.cfg"

echo WARNING: Factory reset erases all Flash contents.
echo Running MSPM0 factory reset...
"%OPENOCD%" -s "%SCRIPTS%" -f "%CONFIG%" -c "init; mspm0_factory_reset; shutdown"

if errorlevel 1 (
    echo.
    echo Factory reset failed.
    pause
    exit /b 1
)

echo.
echo Factory reset completed.
pause
