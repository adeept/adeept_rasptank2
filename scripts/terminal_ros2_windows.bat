@echo off
rem Abre un terminal con ROS 2 y el RaspTank cargados: ros2 topic list,
rem ros2 run rasptank_alumno ej01_avanzar, python mi_programa.py...
call "%~dp0windows\comun.cmd"
pushd "%RT_ROOT%"
"%RT_PIXI%" run --manifest-path "%RT_MANIFEST%" cmd /d /k scripts\windows\en_ros.cmd echo Entorno ROS 2 del RaspTank, dominio %ROS_DOMAIN_ID%. Escribe exit para salir.
