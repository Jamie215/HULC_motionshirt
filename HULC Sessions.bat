@echo off
rem Double-click to open the HULC session app (tools\app.py) in your browser.
rem Keep this window open while you use the app; close it to stop.
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
  py -3 tools\app.py %*
) else (
  python tools\app.py %*
)
if errorlevel 1 (
  echo.
  echo The app did not start. It needs Python 3 with numpy: pip install numpy
  pause
)
