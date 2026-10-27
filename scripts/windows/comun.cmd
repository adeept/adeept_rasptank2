@echo off
rem Shared variables of the Windows launchers (no setlocal: they reach the caller).
for %%R in ("%~dp0..\..") do set "RT_ROOT=%%~fR"
set "RT_MANIFEST=%RT_ROOT%\tools\windows\pixi.toml"
set "RT_PIXI=pixi"
if exist "%RT_ROOT%\windows\pixi.exe" set "RT_PIXI=%RT_ROOT%\windows\pixi.exe"
rem Simulation: 41, like RobotMK2. For a real robot set its domain first:
rem   set ROS_DOMAIN_ID=10
if not defined ROS_DOMAIN_ID set "ROS_DOMAIN_ID=41"
exit /b 0
