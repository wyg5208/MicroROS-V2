@echo off
chcp 65001 >nul
rem Local LLM agent: Ollama / LM Studio talking to the MCP server on localhost:8000
rem Run the pc_side server bat first (it serves ROS2 topics as MCP tools).

cd /d C:\microros-agent
if "%LLM_BASE%"=="" set LLM_BASE=http://127.0.0.1:11434/v1
if "%LLM_MODEL%"=="" set LLM_MODEL=qwen3.5:9b

python -X utf8 agent.py --mcp-url http://127.0.0.1:8000/mcp --llm-base %LLM_BASE% --model %LLM_MODEL% --vision-model %LLM_MODEL% --max-distance 1.5 --max-angle 90
pause
