@echo off
rem Double-click to open the HULC session app in your browser. The first time,
rem it offers to install what the app needs (pip install -r requirements.txt).
rem Keep this window open while you use the app.
cd /d "%~dp0"
set "PY=python"
where py >nul 2>nul && set "PY=py -3"
%PY% --version >nul 2>nul || goto nopython
%PY% -c "import numpy" >nul 2>nul && goto run

echo The app needs a few Python packages (numpy).
set "ANS="
set /p "ANS=Install them now with pip? [Y/n] "
if /i "%ANS%"=="n" exit /b 1
%PY% -m pip install -r requirements.txt || goto installfailed

:run
%PY% tools\app.py %*
if errorlevel 1 goto crashed
exit /b 0

:nopython
echo Python 3 is not installed. Get it from https://www.python.org/downloads/
pause
exit /b 1

:installfailed
echo Install failed (see above).
pause
exit /b 1

:crashed
echo.
echo The app stopped with an error (see above).
pause
exit /b 1
