$p = Get-CimInstance Win32_Process -Filter "Name like '%python%'" |
    Where-Object { $_.CommandLine -like '*car_chat.py*' }
if ($p) { $p | ForEach-Object { Stop-Process -Id $_.ProcessId -Force; "killed chat PID " + $_.ProcessId } }
Start-Sleep -Milliseconds 700
$r = Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{
    CommandLine = 'cmd /c "C:\Python314\python.exe -X utf8 C:\microros-agent\car_chat.py --mcp-url http://127.0.0.1:8000/mcp --llm-base http://127.0.0.1:11434/v1 --model qwen3.5:9b --vision-model qwen3.5:9b --max-distance 1.5 --max-angle 90 >> C:\microros-agent\_chat_server.log 2>&1"'
}
"relaunched chat PID " + $r.ProcessId
