@echo off
setlocal

set "CMAKE=D:\Apps\CLion 2025.3.3\bin\cmake\win\x64\bin\cmake.exe"
set "OPENOCD=D:\DevEnv\ti-openocd\openocd_1.5.0.75\openocd\bin\openocd.exe"
set "OPENOCD_SCRIPTS=D:\DevEnv\ti-openocd\openocd_1.5.0.75\openocd\share\openocd\scripts"
set "OPENOCD_CONFIG=%~dp0..\openocd\mspm0g3519_xds110.cfg"

pushd "%~dp0.."

"%CMAKE%" --build --preset g3519-debug
if errorlevel 1 goto :failed

"%OPENOCD%" -s "%OPENOCD_SCRIPTS%" -f "%OPENOCD_CONFIG%" -c "program build/g3519-debug/app.hex verify reset exit"
if errorlevel 1 goto :failed

echo.
echo XDS110 programming finished.
popd
pause
exit /b 0

:failed
echo.
echo XDS110 build or flash failed. Check the error message above.
popd
pause
exit /b 1
