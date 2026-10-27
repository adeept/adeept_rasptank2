@echo off
rem Simulacion con RViz:  sim_windows.bat [mision^|linea^|vacio^|calibracion^|clasificar] [argumentos de launch]
rem   sim_windows.bat mision mission:=true autostart:=true
setlocal
call "%~dp0windows\comun.cmd"
pushd "%RT_ROOT%"
set "MUNDO=%~1"
if "%MUNDO%"=="" set "MUNDO=mision"
set "ARGS="
:args
shift
if "%~1"=="" goto run
set "ARGS=%ARGS% %1"
goto args
:run
"%RT_PIXI%" run --manifest-path "%RT_MANIFEST%" cmd /d /c scripts\windows\en_ros.cmd ros2 launch rasptank_bringup sim.launch.py world:=%MUNDO%%ARGS%
set "RC=%ERRORLEVEL%"
popd
exit /b %RC%
