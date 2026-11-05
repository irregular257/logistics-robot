@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
if not exist "%~dp0..\.build" mkdir "%~dp0..\.build"
cd /d "%~dp0"
cl /nologo /utf-8 /EHsc /W4 /I "..\..\Core\Inc" /I "..\..\Hardware\duoji" /I "..\..\Hardware\JY61P" lift_guard_host.cpp /Fo"%~dp0..\.build\lift_guard_host.obj" /Fe:"%~dp0..\.build\lift_guard_host.exe"
if errorlevel 1 exit /b 1
"%~dp0..\.build\lift_guard_host.exe"
exit /b %errorlevel%
