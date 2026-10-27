@echo off
rem Permite el trafico de ROS 2 (DDS) con el robot en la red local.
rem Ejecutar como administrador, una vez. /preview: solo muestra las reglas.
setlocal
call "%~dp0windows\comun.cmd"
set "ENV=%RT_ROOT%\tools\windows\.pixi\envs\default"
set "PREVIEW=0"
if /i "%~1"=="/preview" set "PREVIEW=1"
call :rule "RaspTank ROS2 Python" "%ENV%\python.exe"
call :rule "RaspTank ROS2 RViz" "%ENV%\Library\lib\rviz2\rviz2.exe"
call :rule "RaspTank ROS2 State Publisher" "%ENV%\Library\lib\robot_state_publisher\robot_state_publisher.exe"
echo Hecho.
exit /b 0
:rule
if not exist "%~2" (
    echo [OMITIDO] no existe %~2
    exit /b 0
)
if "%PREVIEW%"=="1" (
    echo [PREVIEW] %~1: %~2
    exit /b 0
)
netsh advfirewall firewall delete rule name="%~1" >nul 2>nul
netsh advfirewall firewall add rule name="%~1" dir=in action=allow program="%~2" protocol=any remoteip=localsubnet,127.0.0.1 profile=any enable=yes
exit /b %ERRORLEVEL%
