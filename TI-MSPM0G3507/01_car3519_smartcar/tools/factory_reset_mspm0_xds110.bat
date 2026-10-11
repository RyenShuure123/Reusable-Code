@echo off
setlocal

for %%I in ("%~dp0..") do set "PROJECT_ROOT=%%~fI"
set "OPENOCD=D:\DevEnv\ti-openocd\openocd_1.5.0.75\openocd\bin\openocd.exe"
set "SCRIPTS=D:\DevEnv\ti-openocd\openocd_1.5.0.75\openocd\share\openocd\scripts"
set "CONFIG=%PROJECT_ROOT%\openocd\mspm0g3519_xds110.cfg"

echo WARNING: Factory reset erases all Flash contents.
echo Ensure the XDS110 nRESET signal is connected to the target NRST pin.
echo Running MSPM0 factory reset through XDS110...
"%OPENOCD%" -s "%SCRIPTS%" -f "%CONFIG%" -c "init; mspm0_factory_reset; shutdown"

if errorlevel 1 (
    echo.
    echo XDS110 factory reset failed.
    pause
    exit /b 1
)

echo.
echo XDS110 factory reset completed.
pause
exit /b 0
