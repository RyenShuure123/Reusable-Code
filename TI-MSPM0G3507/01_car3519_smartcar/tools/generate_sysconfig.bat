@echo off
setlocal

set "PROJECT_ROOT=%~dp0.."
set "SYSCONFIG=D:\ti\sysconfig\sysconfig_cli.bat"
set "SDK_ROOT=D:\ti\mspm0_sdk_2_11_00_07"
set "SYSCFG_FILE=%PROJECT_ROOT%\syscfg\car3519_minimal.syscfg"
set "OUTPUT_DIR=%PROJECT_ROOT%\generated"

call "%SYSCONFIG%" --compiler gcc --product "%SDK_ROOT%\.metadata\product.json" --script "%SYSCFG_FILE%" --output "%OUTPUT_DIR%"
if errorlevel 1 (
    echo.
    echo SysConfig generation failed.
    pause
    exit /b 1
)

echo.
echo SysConfig generation completed.
pause
