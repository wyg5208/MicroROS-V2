param([string]$Cmd, [string]$Dir)
$r = Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{CommandLine = $Cmd; CurrentDirectory = $Dir}
Write-Output ("rc=" + $r.ReturnValue + " pid=" + $r.ProcessId)
