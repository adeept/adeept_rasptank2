@echo off
rem Instala ROS 2 Humble (RoboStack, con Pixi) y compila el RaspTank.
rem Necesita internet la primera vez (~3 GB). No hace falta Visual Studio.
call "%~dp0windows\comun.cmd"
where pixi >nul 2>nul
if not errorlevel 1 goto have_pixi
if exist "%RT_ROOT%\windows\pixi.exe" goto have_pixi
echo Descargando pixi.exe en windows\ ...
if not exist "%RT_ROOT%\windows" mkdir "%RT_ROOT%\windows"
powershell.exe -NoProfile -Command "Invoke-WebRequest -UseBasicParsing -Uri 'https://github.com/prefix-dev/pixi/releases/latest/download/pixi-x86_64-pc-windows-msvc.exe' -OutFile '%RT_ROOT%\windows\pixi.exe'"
if errorlevel 1 goto failed
set "RT_PIXI=%RT_ROOT%\windows\pixi.exe"
:have_pixi
echo Instalando el entorno de ROS 2 (tarda unos minutos)...
"%RT_PIXI%" install --manifest-path "%RT_MANIFEST%"
if errorlevel 1 goto failed
call "%~dp0compilar_windows.bat"
if errorlevel 1 goto failed
echo.
echo Listo. Prueba: scripts\sim_windows.bat   o   scripts\app_windows.bat
exit /b 0
:failed
echo.
echo ERROR: la instalacion no ha terminado. Revisa el mensaje anterior.
exit /b 1
