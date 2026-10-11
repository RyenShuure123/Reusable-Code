@echo off
setlocal

set "OPENOCD=D:\DevEnv\ti-openocd\openocd_1.3.1.50\openocd_1.3.1.50\bin\openocd.exe"
set "SCRIPTS=D:\DevEnv\ti-openocd\openocd_1.3.1.50\openocd_1.3.1.50\share\openocd\scripts"
set "HEX=%~dp0..\build\g3507\bmi270_spi_basic.hex"
set "HEX=%HEX:\=/%"

"%OPENOCD%" -s "%SCRIPTS%" ^
    -f interface/cmsis-dap.cfg ^
    -c "adapter speed 20000" ^
    -f target/ti_mspm0.cfg ^
    -c "program \"%HEX%\" reset exit"

if errorlevel 1 (
    echo.
    echo Fast OpenOCD flash failed. Try the verified 10 MHz script instead.
    pause
)
