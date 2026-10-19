# Detached launch of the PC-side bridge (console :8010, MCP :8000).
# cmd /c so the .bat's `>>` redirection happens inside cmd, not in this caller; WMI Create
# detaches it from this console so closing the window does not kill the car link.
$ErrorActionPreference = 'Stop'
$bat = 'C:\microros-agent\pc_side\_web_bridge.bat'
$r = Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{
    CommandLine      = "cmd.exe /c $bat"
    CurrentDirectory = 'C:\microros-agent\pc_side'
}
if ($r.ReturnValue -ne 0) { throw "Create returned $($r.ReturnValue)" }
"bridge started, PID $($r.ProcessId)"
