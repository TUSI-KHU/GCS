@echo off
chcp 65001 >nul
cd /d "%~dp0"
python tools\prepare_web_assets.py || exit /b 1
python app.py --simulate
