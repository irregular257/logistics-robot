@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
if not exist ".build" mkdir ".build"
cl /nologo /std:c++17 /EHsc /W4 /utf-8 /I "..\..\ESP32\libraries\TurntableNow\src" state_machine.cpp /Fo".build\state_machine.obj" /Fe:".build\state_machine.exe"
if errorlevel 1 exit /b 1
".build\state_machine.exe"
exit /b %errorlevel%
