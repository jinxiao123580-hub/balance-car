@echo off
rem Balance-car BLE tuning panel launcher (ASCII only to avoid codepage issues)
cd /d "%~dp0"
if exist "%~dp0device.local.bat" call "%~dp0device.local.bat"
set PYTHONUTF8=1
set PY=%~dp0.venv\Scripts\python.exe
if not exist "%PY%" (
  echo Panel Python environment is missing: %PY%
  echo Recreate .venv and run: .venv\Scripts\python.exe -m pip install -r requirements.txt
  pause
  exit /b 1
)
"%PY%" "%~dp0ble_tune_panel.py"
if errorlevel 1 pause
