@echo off
rem Detached launch of the PC-side bridge with the Windows console on :8010.
call C:\microros-agent\pc_side\_rosrun.bat car_mcp_server.py --ros-domain-id 40 --camera-ip 192.168.31.150 --port 8000 --safe-distance 0.25 --web-port 8010 >> C:\microros-agent\pc_side\_server.log 2>&1
