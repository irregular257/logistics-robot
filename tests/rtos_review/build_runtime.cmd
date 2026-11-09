@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
if not exist ".build" mkdir ".build"
cl /nologo /TC /std:c11 /W4 /I stubs /I "..\..\Core\Inc" /FI "hal_stubs.h" /c "..\..\Core\Src\robot_control.c" /Fo".build\robot_control.obj"
if errorlevel 1 exit /b 1
cl /nologo /TC /std:c11 /W4 /I stubs /I "..\..\Core\Inc" /FI "hal_stubs.h" /c "..\..\Core\Src\motor_bus.c" /Fo".build\motor_bus.obj"
if errorlevel 1 exit /b 1
cl /nologo /std:c++17 /EHs /W4 /utf-8 /I stubs /I "..\..\Core\Inc" runtime_host.cpp ".build\robot_control.obj" ".build\motor_bus.obj" /Fo".build\runtime_host.obj" /Fe:".build\runtime_host.exe"
if errorlevel 1 exit /b 1
".build\runtime_host.exe"
exit /b %errorlevel%
