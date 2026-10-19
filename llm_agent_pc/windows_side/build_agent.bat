@echo off
chcp 65001 >nul
rem Build micro-ROS Agent natively on Windows (Humble ships no prebuilt Windows agent).
rem Needs: VS 2022 BuildTools, C:\opt\ros\humble\x64, Python 3.10 + colcon, GitHub over SSH.

setlocal

set VSBASE=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools
set ROS2_DIR=C:\opt\ros\humble\x64
set PY310=C:\Users\Leo\AppData\Local\Programs\Python\Python310
set PY310EXE=C:/Users/Leo/AppData/Local/Programs/Python/Python310/python.exe
set WS=C:\microros-agent\agent_ws

call "%VSBASE%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
set PATH=%VSBASE%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin;%VSBASE%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja;%PY310%;%PY310%\Scripts;%APPDATA%\Python\Python310\Scripts;%PATH%

call "%ROS2_DIR%\local_setup.bat"
if errorlevel 1 exit /b 1
rem the bundle's setup points python at a non-existent pixi env; use local 3.10
set "_colcon_python_executable=%PY310%\python.exe"

rem SuperBuild clones eProsima deps over https; rewrite to SSH for this process only
set GIT_CONFIG_COUNT=1
set GIT_CONFIG_KEY_0=url.git@github.com:.insteadOf
set GIT_CONFIG_VALUE_0=https://github.com/

cd /d "%WS%"
echo [build] micro_ros_msgs + micro_ros_agent
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=%PY310EXE% -DPYTHON_EXECUTABLE=%PY310EXE%
if errorlevel 1 exit /b 1

echo done: %WS%\install\micro_ros_agent\bin\micro_ros_agent.exe
endlocal
