@echo off
set "CMAKE=D:\Apps\CLion 2025.3.3\bin\cmake\win\x64\bin\cmake.exe"

pushd "%~dp0.."
"%CMAKE%" --build --preset g3519-debug --target flash
set "RESULT=%ERRORLEVEL%"
popd

if not "%RESULT%"=="0" (
    echo.
    echo Build, SysConfig generation, or OpenOCD flash failed. Check the error message above.
    pause
    exit /b %RESULT%
)

echo.
echo Programming finished.
pause
exit /b 0
