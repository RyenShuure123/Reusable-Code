@echo off
set OPENOCD=D:\DevEnv\ti-openocd\openocd_1.3.1.50\openocd_1.3.1.50\bin\openocd.exe
set SCRIPTS=D:\DevEnv\ti-openocd\openocd_1.3.1.50\openocd_1.3.1.50\share\openocd\scripts
set CFG=D:/ti/MyProjects/mspm0g3507_hardware_spi_bmi270/openocd/mspm0g3507_daplink.cfg
set HEX=D:/ti/MyProjects/mspm0g3507_hardware_spi_bmi270/build/g3507/bmi270_spi_basic.hex

"%OPENOCD%" -s "%SCRIPTS%" -f "%CFG%" -c "program \"%HEX%\" verify reset exit"
if errorlevel 1 (
    echo.
    echo OpenOCD flash failed. Check the error message above.
    pause
)
