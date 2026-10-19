@echo off
chcp 65001 >nul
rem micro-ROS Agent (udp/8090): the ESP32 board connects here and its topics
rem become a normal ROS2 graph on this machine.
rem ROS_DOMAIN_ID must match the bridge (40), otherwise topics never show up.
setlocal
set ROS2_DIR=C:\opt\ros\humble\x64
set WS=C:\microros-agent\agent_ws
set ROS_DOMAIN_ID=40

if exist "%WS%\install\micro_ros_agent\lib\micro_ros_agent\micro_ros_agent.exe" goto :run
echo [ERROR] agent not built yet, run agent_ws\build_agent.bat first
pause
exit /b 1

:run
call "%ROS2_DIR%\local_setup.bat" >nul 2>nul
call "%WS%\install\local_setup.bat" >nul 2>nul
set "_colcon_python_executable=C:\Users\Leo\AppData\Local\Programs\Python\Python310\python.exe"
echo [agent] ROS_DOMAIN_ID=%ROS_DOMAIN_ID% udp/8090
"%WS%\install\micro_ros_agent\lib\micro_ros_agent\micro_ros_agent.exe" udp4 --port 8090
pause
