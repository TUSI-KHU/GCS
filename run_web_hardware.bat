@echo off
chcp 65001 >nul
cd /d "%~dp0"
if "%~1"=="" (
  echo Usage: run_web_hardware.bat COM_PORT
  echo Example: run_web_hardware.bat COM5
  exit /b 2
)
python tools\prepare_web_assets.py || exit /b 1
python app.py --serial-port "%~1" --serial-mode raw
