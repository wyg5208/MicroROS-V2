@echo off
chcp 65001 >nul
rem Install the car scheduler. Task Scheduler wakes car_cron.py once a minute; the script decides
rem whether a job is due, and a due job only POSTs one sentence to the chat session on :8020.
rem No ROS here, so no _rosrun.bat and no admin rights.
rem
rem Running this file cannot move the car: cron_tasks.json ships with "enabled": false on every
rem task, so the poller wakes, looks, and goes back to sleep. Open a task yourself, only then
rem does anything get sent.
rem
rem To pause everything without uninstalling: create an empty file named cron.pause in this folder.
rem To stop a turn that is already running: the stop button on the :8020 page.
rem
rem Comments stay ASCII on purpose: cmd.exe tracks this file by byte offset, a multibyte line can
rem split mid-character and the remainder gets run as a command (observed 2026-10-19).
setlocal
set TASK=MicroROS-CarCron
set PYEXE=C:\Python314\python.exe

rem Pin the folder the task runs from. Double-clicking the copy that lives in the G-drive mirror
rem would otherwise register a task against that mirror and write the log into it, so the running
rem copy and the edited copy would drift apart. C:\microros-agent wins when it is there.
set BASE=C:\microros-agent\
if not exist "%BASE%car_cron.py" set BASE=%~dp0

if not exist "%PYEXE%" (
  echo C:\Python314\python.exe not found, falling back to python on PATH.
  set PYEXE=python
)

rem Paths must stay free of spaces: schtasks stores this line as text and splits it again when the
rem task runs, so a folder like C:\My Cars\ makes the command line nonsense. This folder is
rem C:\microros-agent, which is why it works. Verify later with: schtasks /Query /TN %TASK% /V /FO LIST
schtasks /Create /F /TN "%TASK%" /SC MINUTE /MO 1 /TR "cmd /c %PYEXE% -X utf8 %BASE%car_cron.py --once 1>> %BASE%_cron_task.log 2>&1"
if errorlevel 1 (
  echo schtasks refused the task -- read its message above.
  pause
  exit /b 1
)

rem Task Scheduler's defaults say "do not start on battery" and "never make up a missed run". This
rem PC is a laptop and the 2026-10-19 voice test was driven with the charger unplugged, so under
rem those defaults a job would silently never fire -- silently, because the task is never woken and
rem car_cron.py gets no chance to write a reason into _cron.log. schtasks' command line does not
rem expose these three switches; the scheduledtask cmdlets do. Reading the flags back below is the
rem proof they actually landed, not a hope.
powershell -NoProfile -Command "$s=(Get-ScheduledTask -TaskName %TASK%).Settings; $s.DisallowStartIfOnBatteries=$false; $s.StopIfGoingOnBatteries=$false; $s.StartWhenAvailable=$true; Set-ScheduledTask -TaskName %TASK% -Settings $s | Out-Null; $n=(Get-ScheduledTask -TaskName %TASK%).Settings; Write-Output ('power flags now:  start-on-battery=' + (-not $n.DisallowStartIfOnBatteries) + '  keep-running-on-battery=' + (-not $n.StopIfGoingOnBatteries) + '  make-up-missed-run=' + $n.StartWhenAvailable)"
if errorlevel 1 (
  echo Could not flip the power flags. The task still runs while the charger is plugged in, but it
  echo will NOT start on battery. Fix by hand: Task Scheduler - this task - Conditions - uncheck
  echo "Start the task only if the computer is on AC power".
)

echo.
echo Registered %TASK%: car_cron.py --once runs every minute.
echo   job table   %BASE%cron_tasks.json
echo   its log     %BASE%_cron.log   (why a job fired / was skipped)
echo               %BASE%_cron_task.log   (crash output, empty when nothing crashed)
echo   results     the :8020 chat page, same as if you had typed the sentence yourself
echo.
echo Check the alarm clock before trusting it:  python -X utf8 %BASE%car_cron.py --list
echo Uninstall with the other bat in this folder (it only deletes the scheduled task, the job
echo table and both logs stay on disk).
pause
