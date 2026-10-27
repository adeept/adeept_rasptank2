@echo off
rem Aplicacion del RaspTank:  app_windows.bat [--dominio 10]
call "%~dp0windows\comun.cmd"
pushd "%RT_ROOT%"
"%RT_PIXI%" run --manifest-path "%RT_MANIFEST%" cmd /d /c scripts\windows\en_ros.cmd python -m rasptank_app %*
set "RC=%ERRORLEVEL%"
popd
exit /b %RC%
