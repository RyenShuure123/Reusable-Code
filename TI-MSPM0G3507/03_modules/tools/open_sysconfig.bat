@echo off
set SYSCONFIG=D:\ti\sysconfig\sysconfig_gui.bat
set SDK_ROOT=D:\ti\mspm0_sdk_2_11_00_07

if "%~1"=="" (
    "%SYSCONFIG%" --product "%SDK_ROOT%\.metadata\product.json"
) else (
    "%SYSCONFIG%" --product "%SDK_ROOT%\.metadata\product.json" --script "%~1"
)

