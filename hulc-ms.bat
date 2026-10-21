@echo off
rem The HULC-MS app and commands, from this repository (Python 3.11-3.13).
rem   Double-click: opens the HULC-MS app in your browser.
rem   In a terminal here: hulc-ms nodes offload ... / hulc-ms analyze ... (see README).
rem The first time, it offers to install what it needs (pip install -r requirements.txt).
cd /d "%~dp0"
set "PY=python"
where py >nul 2>nul && set "PY=py -3"
%PY% --version >nul 2>nul || goto nopython
%PY% -c "import numpy" >nul 2>nul && goto run

echo hulc-ms needs a few Python packages (numpy, bleak).
set "ANS="
set /p "ANS=Install them now with pip? [Y/n] "
if /i "%ANS%"=="n" exit /b 1
%PY% -m pip install -r requirements.txt || goto installfailed

:run
%PY% hulc-ms.py %*
if errorlevel 1 goto failed
exit /b 0

:nopython
echo Python 3 is not installed. Get it from https://www.python.org/downloads/
echo (or download the standalone HULC-MS app, which needs no Python - see README)
if "%~1"=="" pause
exit /b 1

:installfailed
echo Install failed (see above).
if "%~1"=="" pause
exit /b 1

:failed
if not "%~1"=="" exit /b 1
echo.
echo The app stopped with an error (see above).
pause
exit /b 1
