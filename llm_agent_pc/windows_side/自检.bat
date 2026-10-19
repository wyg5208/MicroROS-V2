@echo off
chcp 65001 >nul
rem PC-side link check: camera frame reachable + car ROS topics online
setlocal
set ROS2_DIR=C:\opt\ros\humble\x64
set PY310=C:\Users\Leo\AppData\Local\Programs\Python\Python310\python.exe
if "%CAR_CAMERA_IP%"=="" set CAR_CAMERA_IP=192.168.31.150

call "%ROS2_DIR%\local_setup.bat" >nul 2>nul
set "_colcon_python_executable=%PY310%"
cd /d C:\microros-agent\pc_side
"%PY310%" -X utf8 selfcheck.py %CAR_CAMERA_IP%
pause
