@echo off
rem Allow the car's micro-ROS client to reach the Agent on this laptop: UDP 8090 inbound.
rem Windows Firewall blocks unsolicited inbound by default, so a fresh machine gives the
rem nastiest possible symptom: the Agent runs, the car joins the same WiFi, and not one ROS
rem topic ever appears. This adds one named rule and is safe to run twice.
set RULE=microros-agent-udp-8090

net session >nul 2>&1
if errorlevel 1 (
  echo [*] this needs administrator rights, opening an elevated window ...
  powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
  exit /b
)

netsh advfirewall firewall show rule name=%RULE% | find "%RULE%" >nul
if errorlevel 1 goto :add
echo [=] rule already present, replacing it with a clean copy
netsh advfirewall firewall delete rule name=%RULE% >nul

:add
echo [+] adding inbound allow: UDP 8090
netsh advfirewall firewall add rule name=%RULE% dir=in action=allow protocol=UDP localport=8090 enable=yes profile=any
echo.
echo Current state:
netsh advfirewall firewall show rule name=%RULE%
echo.
echo Re-check from the PC side: run the self-check batch in this folder
echo To undo:  netsh advfirewall firewall delete rule name=%RULE%
pause
