@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
cl /nologo /utf-8 /EHsc /W4 /I stubs servo_pwm.cpp "..\..\Hardware\duoji\duiji.c" /Fe:servo_pwm.exe
if errorlevel 1 exit /b 1
servo_pwm.exe
exit /b %errorlevel%
