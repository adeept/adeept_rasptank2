@echo off
rem Compila el workspace (todo Python: sin compilador de C++).
call "%~dp0windows\comun.cmd"
pushd "%RT_ROOT%\ros2_ws"
"%RT_PIXI%" run --manifest-path "%RT_MANIFEST%" colcon build --merge-install
set "RC=%ERRORLEVEL%"
popd
exit /b %RC%
