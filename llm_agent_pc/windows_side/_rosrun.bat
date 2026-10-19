@echo off
setlocal
set ROS2_DIR=C:\opt\ros\humble\x64
set PY310=C:\Users\Leo\AppData\Local\Programs\Python\Python310\python.exe
set ROS_DOMAIN_ID=40
call "%ROS2_DIR%\local_setup.bat" >nul 2>nul
set "_colcon_python_executable=%PY310%"
cd /d C:\microros-agent\pc_side
"%PY310%" -X utf8 %*
endlocal
