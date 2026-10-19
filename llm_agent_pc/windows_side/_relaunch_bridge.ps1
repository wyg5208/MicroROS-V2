# Detached relaunch of the PC-side bridge (MCP :8000 + console :8010).
# WMI Win32_Process.Create rather than Start-Process: a child of this shell would die with it.
$r = Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{
    CommandLine      = 'cmd /c C:\microros-agent\pc_side\_web_bridge.bat'
    CurrentDirectory = 'C:\microros-agent\pc_side'
}
"ReturnValue=$($r.ReturnValue) PID=$($r.ProcessId)"
