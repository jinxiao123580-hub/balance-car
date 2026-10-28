@echo off
setlocal
cd /d "%~dp0"
title Balance Car BLE Telemetry

echo ============================================
echo   Balance Car BLE Telemetry Launcher
echo ============================================

REM ---- find a working Python (py / python / PlatformIO python) ----
set "PY="
py -c "print(1)" >nul 2>nul
if not errorlevel 1 set "PY=py"
if not defined PY (
  python -c "print(1)" >nul 2>nul
  if not errorlevel 1 set "PY=python"
)
if not defined PY (
  if exist "%USERPROFILE%\.platformio\penv\Scripts\python.exe" set "PY=%USERPROFILE%\.platformio\penv\Scripts\python.exe"
)
if not defined PY (
  echo [ERROR] No working Python found.
  echo         Install Python 3.10+ from https://www.python.org/downloads/
  echo         and tick "Add Python to PATH", then run this again.
  pause
  exit /b 1
)
echo Using Python: %PY%
%PY% --version

REM ---- install dependencies on first run ----
%PY% -c "import bleak, matplotlib" >nul 2>nul
if errorlevel 1 (
  echo.
  echo First run: installing bleak + matplotlib, please wait a few minutes...
  %PY% -m pip install -U bleak matplotlib
  if errorlevel 1 (
    echo.
    echo [ERROR] pip install failed. Check your network and retry.
    pause
    exit /b 1
  )
)

REM ---- ensure a GUI toolkit so the plot window can show ----
%PY% -c "import PyQt5" >nul 2>nul
if errorlevel 1 (
  %PY% -c "import tkinter" >nul 2>nul
  if errorlevel 1 (
    echo.
    echo No GUI toolkit found. Installing PyQt5, one-time download.
    %PY% -m pip install PyQt5
    if errorlevel 1 (
      echo [WARN] PyQt5 install failed. The plot window may not show,
      echo        but data will still be saved to CSV.
    )
  )
)

REM ---- run the plotter ----
echo.
echo Connecting to BLE module... close the plot window to stop.
%PY% "%~dp0ble_telemetry_plot.py" %*
echo.
echo Finished. CSV log saved next to this file (ble_telemetry_log.csv).
pause
