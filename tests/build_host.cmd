@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
if not exist "%~dp0.build" mkdir "%~dp0.build"
cl /nologo /LD /TC /std:c11 /W4 /I "%~dp0..\Core\Inc" /I "%~dp0..\HARDWARE\JY61P" /Fo"%~dp0.build\vision_follow_host.obj" "%~dp0follow_host.c" /link /OUT:"%~dp0.build\vision_follow_host.dll" /IMPLIB:"%~dp0.build\vision_follow_host.lib"
