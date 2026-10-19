@echo off
chcp 65001 >nul
rem Remove the car scheduler. This does NOT stop a turn that is already running -- for that use the
rem stop button on the :8020 page. It also does not delete anything: the job table, both logs and
rem cron.pause are plain files next to this bat and stay exactly where they are.
rem
rem If you only want a holiday, leave the task installed and create an empty file named cron.pause
rem in this folder: the poller then wakes, sees the file, sends nothing, and says so once a day
rem in _cron.log. Delete the file to resume.
rem
rem Comments stay ASCII on purpose: cmd.exe tracks this file by byte offset, a multibyte line can
rem split mid-character and the remainder gets run as a command (observed 2026-10-19).
setlocal
set TASK=MicroROS-CarCron
schtasks /Query /TN "%TASK%" >nul 2>&1
if errorlevel 1 (
  echo No task named %TASK% is registered, so there is nothing to remove.
  pause
  exit /b 0
)
schtasks /Delete /F /TN "%TASK%"
if errorlevel 1 (
  echo schtasks refused to delete it -- read its message above.
) else (
  echo Deleted. Nothing else was touched.
)
pause
