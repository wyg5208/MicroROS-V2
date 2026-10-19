@echo off
rem Mirror the build-time paths the pixi-based Humble bundle hard-codes into its
rem exported CMake targets, using this machine's real Python 3.10 / numpy 2.x.
setlocal
set PIX=C:\pixi_ws\.pixi\envs\default
set PY310=C:\Users\Leo\AppData\Local\Programs\Python\Python310
set NP=%PY310%\lib\site-packages\numpy

if exist "%PIX%\libs\python310.lib" goto :lib_done
mkdir "%PIX%\libs" 2>nul
copy /Y "%PY310%\libs\python310.lib" "%PIX%\libs\python310.lib" >nul
:lib_done

if exist "%PIX%\lib\site-packages\numpy\core\include\numpy" goto :np_done
mkdir "%PIX%\lib\site-packages\numpy\core" 2>nul
rmdir "%PIX%\lib\site-packages\numpy\core\include" 2>nul
mklink /J "%PIX%\lib\site-packages\numpy\core\include" "%NP%\_core\include"
:np_done

echo mirror ready:
dir /b "%PIX%\libs"
dir /b "%PIX%\lib\site-packages\numpy\core\include"
endlocal
