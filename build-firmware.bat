@echo off
setlocal
cd /d "%~dp0software\firmware"
if exist "%USERPROFILE%\.platformio\penv\Scripts\pio.exe" (
  "%USERPROFILE%\.platformio\penv\Scripts\pio.exe" run -e black_f407zg
) else (
  pio run -e black_f407zg
)
if errorlevel 1 pause
