@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
cl /nologo /EHsc /W4 /utf-8 /I stubs /I "..\..\ESP32\libraries\TurntableNow\src" bridge_host.cpp /Fe:bridge_host.exe
if errorlevel 1 exit /b 1
bridge_host.exe
exit /b %errorlevel%
