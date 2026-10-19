@echo off
chcp 65001 >nul
setlocal
set PY310=C:\Users\Leo\AppData\Local\Programs\Python\Python310\python.exe
call C:\opt\ros\humble\x64\local_setup.bat
echo PYTHONPATH=%PYTHONPATH%
%PY310% -c "import rclpy,sys;print('rclpy OK', sys.version.split()[0], rclpy.__file__)"
%PY310% -c "from geometry_msgs.msg import Twist, Vector3; from nav_msgs.msg import Odometry; from sensor_msgs.msg import LaserScan; from std_msgs.msg import UInt16; print('msg types OK')"
%PY310% -c "import mcp; print('mcp OK', mcp.__version__ if hasattr(mcp,'__version__') else '')"
