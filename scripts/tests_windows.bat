@echo off
rem Tests unitarios y de la app; con --e2e, tambien la mision simulada.
call "%~dp0windows\comun.cmd"
pushd "%RT_ROOT%"
"%RT_PIXI%" run --manifest-path "%RT_MANIFEST%" cmd /d /c scripts\windows\en_ros.cmd python scripts\tests.py %*
set "RC=%ERRORLEVEL%"
popd
exit /b %RC%
