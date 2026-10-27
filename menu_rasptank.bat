@echo off
rem Menu del RaspTank en Windows.
setlocal
cd /d "%~dp0"
:menu
cls
echo ==================== RaspTank ====================
echo  1. Instalar / actualizar ROS 2 (solo la primera vez)
echo  2. Compilar el workspace
echo  3. Aplicacion (estado, parada, mover, calibracion)
echo  4. Simulacion: mision
echo  5. Simulacion: seguir linea
echo  6. Simulacion: calibracion
echo  7. Simulacion: clasificar cubos
echo  8. Terminal de ROS 2 (programas del alumno)
echo  9. Tests
echo  0. Salir
echo ==================================================
set "OPT="
set /p "OPT=Elige: "
if "%OPT%"=="1" call scripts\instalar_windows.bat
if "%OPT%"=="2" call scripts\compilar_windows.bat
if "%OPT%"=="3" start "RaspTank app" cmd /c scripts\app_windows.bat
if "%OPT%"=="4" start "Simulacion" cmd /c scripts\sim_windows.bat mision
if "%OPT%"=="5" start "Simulacion" cmd /c scripts\sim_windows.bat linea
if "%OPT%"=="6" start "Simulacion" cmd /c scripts\sim_windows.bat calibracion
if "%OPT%"=="7" start "Simulacion" cmd /c scripts\sim_windows.bat clasificar sort:=true mission:=true
if "%OPT%"=="8" start "ROS 2" cmd /c scripts\terminal_ros2_windows.bat
if "%OPT%"=="9" call scripts\tests_windows.bat
if "%OPT%"=="0" exit /b 0
pause
goto menu
