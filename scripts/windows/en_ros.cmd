@echo off
rem Runs a command with ROS 2 and the RaspTank workspace loaded.
rem It is called through "pixi run", which has already activated ROS 2.
if exist "%RT_ROOT%\ros2_ws\install\setup.bat" (
    call "%RT_ROOT%\ros2_ws\install\setup.bat"
) else (
    echo AVISO: el workspace no esta compilado. Ejecuta scripts\compilar_windows.bat
)
set "PYTHONPATH=%RT_ROOT%\tools\app;%PYTHONPATH%"
%*
exit /b %ERRORLEVEL%
