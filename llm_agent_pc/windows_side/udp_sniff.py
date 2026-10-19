"""监听 8090/udp，确认车控板的 micro-ROS 发现包是否到达本机（agent 未启动时用）。"""
import socket
import sys
import time

port = int(sys.argv[1]) if len(sys.argv) > 1 else 8090
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", port))
s.settimeout(1.0)
print("监听 udp/%d %.0f 秒 ..." % (port, secs))
n = 0
t0 = time.time()
while time.time() - t0 < secs:
    try:
        data, addr = s.recvfrom(2048)
    except socket.timeout:
        continue
    n += 1
    print("  %s -> %d bytes  %s" % (addr[0], len(data), data[:24].hex()))
print("共收到 %d 个包%s" % (n, "" if n else "（车没发过来：查 WiFi 是否连上、agent_ip 是否为本机 IP）"))
