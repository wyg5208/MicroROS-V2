@echo off
chcp 65001 >nul
rem MicroROS-V2 occupancy grid mapper: /scan + /odom_raw -> cache\map.png (+ map.json)
rem Needs: Agent running on udp/8090, car and laptop on the same WiFi.
rem Stop with Ctrl-C, or just close the window; map.png is rewritten every 2 seconds.

setlocal
set ROS2_DIR=C:\opt\ros\humble\x64
set PY310=C:\Users\Leo\AppData\Local\Programs\Python\Python310\python.exe

if exist "%ROS2_DIR%\local_setup.bat" goto :run
echo [ERROR] missing %ROS2_DIR%\local_setup.bat
pause
exit /b 1

:run
call "%ROS2_DIR%\local_setup.bat" >nul 2>nul
set "_colcon_python_executable=%PY310%"
cd /d C:\microros-agent\pc_side
"%PY310%" -X utf8 mapper.py --ros-domain-id 40 --out-dir cache
pause
