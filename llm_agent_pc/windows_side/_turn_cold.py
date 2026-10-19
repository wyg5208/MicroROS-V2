# -*- coding: utf-8 -*-
"""Arm A of the +10.8 deg outlier hunt: 10 consecutive 30 deg turns, landing measured
independently from /api/state pose_raw (not from what Move reports).

`--cold` runs arm B: kill and relaunch the bridge first, then fire the turns as soon as the
console answers -- the outlier was only ever seen on the first two commands after a restart, so
the gap between "odom is fresh again" and "first Move" has to stay a few hundred ms, not seconds.

Mapping is switched off for the run so these spins cannot add more smear to a map that is
already the documented casualty of iteration 3.
"""
import json
import math
import os
import subprocess
import sys
import time
import urllib.request

sys.path.insert(0, r"C:\microros-agent")
from agent import McpClient  # noqa: E402

CONSOLE = "http://127.0.0.1:8010"
ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
COLD = "--cold" in sys.argv[1:]
N = int(ARGS[0]) if ARGS else 10
DEG = float(ARGS[1]) if len(ARGS) > 1 else 30.0
SIDE = ARGS[2] if len(ARGS) > 2 else "turn_right"
PS = r"C:\microros-agent\pc_side"


def state():
    return json.loads(urllib.request.urlopen(CONSOLE + "/api/state", timeout=3).read())


def post(path, obj):
    req = urllib.request.Request(CONSOLE + path, data=json.dumps(obj).encode(),
                                 headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=5).read())


def yaw():
    return math.radians(state()["pose_raw"][2])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def restart_bridge():
    """Kill the bridge, relaunch it detached, and return when odom is fresh again.

    The wait is on odom_age, not on "the port answers": the console answers as soon as the HTTP
    thread is up, which is before /odom_raw has any subscriber data -- firing into that window
    would test a different thing than the recorded outlier did.
    """
    def run(script):
        subprocess.run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
                        "-File", os.path.join(PS, script)], check=False)
    run("_kill_bridge.ps1")
    for _ in range(40):
        try:
            urllib.request.urlopen(CONSOLE + "/api/state", timeout=1).read()
            time.sleep(0.25)
        except Exception:                                      # noqa: BLE001
            break
    run("_relaunch_bridge.ps1")
    t0 = time.time()
    while time.time() - t0 < 90:
        try:
            st = state()
            if st.get("odom_age") is not None and st["odom_age"] < 0.3:
                return time.time() - t0, st
        except Exception:                                      # noqa: BLE001
            pass
        time.sleep(0.2)
    raise RuntimeError("bridge did not come back within 90 s")


if COLD:
    up_in, _ = restart_bridge()
    print("cold: bridge back with fresh odom after %.2f s" % up_in)

st = state()
print("before: pose_raw=%s volts=%s samples=%s mapping=%s busy=%r"
      % (st["pose_raw"], st.get("volts"), st["samples"], st["mapping"], st["busy"]))
was_mapping = st["mapping"]
post("/api/mapping", {"on": False})
print("mapping -> %s" % state()["mapping"])

c = McpClient("http://127.0.0.1:8000/mcp")
t_up = time.time()
try:
    for i in range(N):
        t0 = time.time()
        p0 = yaw()
        r = c.call_tool("Move", {"direction": SIDE, "angle": DEG})
        txt = (r if isinstance(r, str) else json.dumps(r, ensure_ascii=False))[:120]
        p1 = yaw()
        got = abs(math.degrees(wrap(p1 - p0)))
        print("%2d  t=%5.2fs  since_up=%5.1fs  net=%+7.2f deg  dev=%+6.2f  |  %s"
              % (i + 1, time.time() - t0, t0 - t_up,
                 math.degrees(wrap(p1 - p0)), got - DEG, txt.replace("\n", " ")))
        time.sleep(0.6)
finally:
    post("/api/mapping", {"on": bool(was_mapping)})
    print("mapping restored -> %s" % state()["mapping"])
