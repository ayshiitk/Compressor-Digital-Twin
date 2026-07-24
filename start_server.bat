@echo off
title G^&M Digital Twin - Live Server
cd /d "%~dp0"

echo Checking for required Python packages (numpy, matplotlib, websockets)...
python -c "import numpy, matplotlib, websockets" 2>NUL
if errorlevel 1 (
    echo Installing missing packages ...
    python -m pip install numpy matplotlib websockets
)

echo.
echo Starting simulation server on ws://localhost:8765 ...
echo Leave this window open while you use the dashboard.
echo.

start "" "dashboard.html"
python server.py

pause