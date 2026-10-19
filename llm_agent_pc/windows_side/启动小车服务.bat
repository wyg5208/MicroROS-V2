@echo off
chcp 65001 >nul
rem MicroROS-V2 PC-side MCP server: ROS2 topics <-> LLM tools on localhost:8000
rem Needs: Agent running on udp/8090, car and laptop on the same WiFi

setlocal
set ROS2_DIR=C:\opt\ros\humble\x64
set PY310=C:\Users\Leo\AppData\Local\Programs\Python\Python310\python.exe
if "%CAR_CAMERA_IP%"=="" set CAR_CAMERA_IP=192.168.31.150

if exist "%ROS2_DIR%\local_setup.bat" goto :run
echo [ERROR] missing %ROS2_DIR%\local_setup.bat
pause
exit /b 1

:run
call "%ROS2_DIR%\local_setup.bat" >nul 2>nul
set "_colcon_python_executable=%PY310%"
cd /d C:\microros-agent\pc_side
"%PY310%" -X utf8 car_mcp_server.py --ros-domain-id 40 --camera-ip %CAR_CAMERA_IP% --port 8000 --safe-distance 0.25
pause
