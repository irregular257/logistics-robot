@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
cl /nologo /EHsc /W3 /utf-8 /I esp_stubs esp32_api.cpp /Fe:esp32_api.exe
if errorlevel 1 exit /b 1
esp32_api.exe
exit /b %errorlevel%
