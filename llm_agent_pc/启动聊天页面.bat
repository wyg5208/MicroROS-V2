@echo off
chcp 65001 >nul
rem Browser chat front-end: talk to the local model, watch each step, approve it (http://127.0.0.1:8020)
rem Order: 1) the Agent launcher bat  2) _web_bridge.bat (MCP :8000 + console :8010)  3) double-click this file
rem This layer never touches ROS, so PATH python is enough -- no _rosrun.bat needed.
rem Comments stay ASCII on purpose: cmd.exe tracks this file by byte offset, so a multibyte
rem line can split mid-character and the remainder gets run as a command (observed 2026-10-19).

cd /d %~dp0
if "%LLM_BASE%"=="" set LLM_BASE=http://127.0.0.1:11434/v1
if "%LLM_MODEL%"=="" set LLM_MODEL=qwen3.5:9b

start "" /min cmd /c "ping -n 3 127.0.0.1 >nul & start "" http://127.0.0.1:8020/"
python -X utf8 car_chat.py --mcp-url http://127.0.0.1:8000/mcp --llm-base %LLM_BASE% --model %LLM_MODEL% --vision-model %LLM_MODEL% --max-distance 1.5 --max-angle 90
if errorlevel 1 pause
