@echo off
set OPENOCD=D:\DevEnv\ti-openocd\openocd_1.3.1.50\openocd_1.3.1.50\bin\openocd.exe
set SCRIPTS=D:\DevEnv\ti-openocd\openocd_1.3.1.50\openocd_1.3.1.50\share\openocd\scripts
set CFG=%~dp0..\openocd\mspm0g3507_daplink.cfg

"%OPENOCD%" -s "%SCRIPTS%" -f "%CFG%" -c "mspm0_factory_reset" -c "shutdown"

if errorlevel 1 (
    echo.
    echo Factory reset failed. Check the NRST, SWD, GND and Vref connections.
    pause
)
