@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
cl /nologo /EHsc /W4 /I stubs /I "..\..\Core\Inc" stm32_sequences.cpp /Fe:stm32_sequences.exe
if errorlevel 1 exit /b 1
stm32_sequences.exe
exit /b %errorlevel%
