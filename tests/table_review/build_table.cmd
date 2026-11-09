@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
if not exist ".build" mkdir ".build"
cl /nologo /utf-8 /EHsc /W3 /I "..\..\Core\Inc" /I "..\..\Hardware\duoji" /I "..\..\Hardware\JY61P" table_host.cpp /Fo".build\table_host.obj" /Fe:".build\table_host.exe"
if errorlevel 1 exit /b 1
".build\table_host.exe"
if errorlevel 1 exit /b 1
cl /nologo /utf-8 /EHsc /W3 /I "..\web_review\stubs" /I "..\..\Core\Inc" table_uart_host.cpp /Fo".build\table_uart_host.obj" /Fe:".build\table_uart_host.exe"
if errorlevel 1 exit /b 1
".build\table_uart_host.exe"
exit /b %errorlevel%
