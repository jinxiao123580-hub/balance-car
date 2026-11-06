@echo off
setlocal
rem 编译并烧录（ST-Link）。构建目录已改到 %USERPROFILE%\.platformio\build\pinghengche，
rem 以避开中文路径导致 ld 无法创建 firmware.map 的问题。
cd /d "%~dp0software\firmware"
if exist "%USERPROFILE%\.platformio\penv\Scripts\pio.exe" (
  "%USERPROFILE%\.platformio\penv\Scripts\pio.exe" run -e black_f407zg -t upload
) else (
  pio run -e black_f407zg -t upload
)
if errorlevel 1 (
  echo.
  echo 烧录失败。常见原因：
  echo   1) ST-Link 没插好或没装驱动（STSW-LINK009）；
  echo   2) 板子没供电，或 SWD 四根线（3V3/GND/SWDIO/SWCLK）接反；
  echo   3) 另一个 openocd/ST-LINK Utility 正占用调试口。
  pause
)
endlocal
