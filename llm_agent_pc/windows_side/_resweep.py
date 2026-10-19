"""Drive the /api/calib sweep with bursts long enough to leave the ramp phase, then fit.

Usage:  _rosrun.bat _resweep.py            # sweep + read-only fit
        _rosrun.bat _resweep.py apply      # sweep + fit + 保存并生效
"""
import json
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8010"


def post(path, body):
    req = urllib.request.Request(BASE + path, data=json.dumps(body).encode("utf-8"),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read().decode("utf-8", "ignore"))


PLAN = [
    (0.10, 0.0, 7.0), (0.12, 0.0, 7.0), (-0.12, 0.0, 7.0),
]
for a in (0.20, 0.35, 0.55, 0.75):
    PLAN.append((0.0, a, 6.0))
for a in (-0.20, -0.35, -0.55, -0.75):
    PLAN.append((0.0, a, 6.0))


def state():
    with urllib.request.urlopen(BASE + "/api/state", timeout=10) as r:
        d = json.loads(r.read().decode("utf-8", "ignore"))
    return d.get("volts"), d.get("pose_raw")


apply_fit = len(sys.argv) > 1 and sys.argv[1] == "apply"
v, p = state()
print("start  volts=%s pose=%s" % (v, p))
for vx, wz, secs in PLAN:
    t = time.strftime("%H:%M:%S")
    try:
        r = post("/api/calib", {"vx": vx, "wz": wz, "seconds": secs})
        print("%s  vx=%+.2f wz=%+.2f  %s" % (t, vx, wz, r.get("text", r)))
    except Exception as e:  # noqa: BLE001 - report and keep going
        print("vx=%+.2f wz=%+.2f  ERROR %s" % (vx, wz, e))
    time.sleep(1.5)
v, p = state()
print("end    volts=%s pose=%s" % (v, p))
out = post("/api/fit", {"apply": apply_fit})
print(json.dumps(out, ensure_ascii=False, sort_keys=True, default=str))
