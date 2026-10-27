@echo off
rem Calibracion de la odometria en el terminal:  calibrar_windows.bat [--auto]
call "%~dp0windows\comun.cmd"
pushd "%RT_ROOT%"
"%RT_PIXI%" run --manifest-path "%RT_MANIFEST%" cmd /d /c scripts\windows\en_ros.cmd ros2 run rasptank_calibration calibrar %*
set "RC=%ERRORLEVEL%"
popd
exit /b %RC%
