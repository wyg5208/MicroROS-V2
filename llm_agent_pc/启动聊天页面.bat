@echo off
chcp 65001 >nul
rem 网页聊天前端：在浏览器里跟大模型说话、看它每一步、替它点头（http://127.0.0.1:8020）
rem 顺序：① 启动Agent.bat  ② _web_bridge.bat（MCP :8000 + 控制台 :8010）  ③ 双击本文件
rem 这一层不碰 ROS，所以用 PATH 里的 python 就行，不需要 _rosrun.bat。

cd /d %~dp0
if "%LLM_BASE%"=="" set LLM_BASE=http://127.0.0.1:11434/v1
if "%LLM_MODEL%"=="" set LLM_MODEL=qwen3.5:9b

start "" /min cmd /c "ping -n 3 127.0.0.1 >nul & start "" http://127.0.0.1:8020/"
python -X utf8 car_chat.py --mcp-url http://127.0.0.1:8000/mcp --llm-base %LLM_BASE% --model %LLM_MODEL% --vision-model %LLM_MODEL% --max-distance 1.5 --max-angle 90
if errorlevel 1 pause
