# -*- coding: utf-8 -*-
"""Task 4 on the real car: lie about the pose *while the wheels are turning* and see whether the
moving detector pulls the motors.

_why_mismatch.py adjudicated a stationary disagreement; _motion_test.py proves the branch logic
offline. This is the one thing neither can say: that a Move which is mid-corridor, with a pose
that has silently become wrong by 1.1 m sideways, actually stops. Nothing here touches the map by
hand -- the corruption is injected the same way a real carry would show up (the pose says one
place, the lidar says another), and recovery is expected to be the machine's own: the abort stops
the car, then the stopped-car detector gets a standing sweep and relocates back.

Run: python _abort_test.py            (plain python: HTTP + the MCP endpoint only)
"""
import json
import math
import os
import shutil
import subprocess
import sys
import time
import urllib.request

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

WEB = "http://127.0.0.1:8010"
CACHE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cache_console")
DIST = float(sys.argv[2]) if len(sys.argv) > 2 else 1.10   # m sideways: _predict_abort.py measured zero 49%, peak 96% at the true pose
PY = sys.executable


def get(path):
    with urllib.request.urlopen(WEB + path, timeout=10) as r:
        return json.loads(r.read().decode("utf-8"))


def post(path, body):
    req = urllib.request.Request(WEB + path, data=json.dumps(body).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=20) as r:
        return json.loads(r.read().decode("utf-8"))


def move(distance, direction):
    """Start a Move through the MCP endpoint and do not wait for it: the point is to corrupt the
    pose while it is still running."""
    args = json.dumps({"direction": direction, "distance": distance})
    return subprocess.Popen([PY, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                              "_mcp_call.py"), "Move", args],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            cwd=os.path.dirname(os.path.abspath(__file__)))


def main():
    stamp = time.strftime("%H%M%S")
    for f in ("grid.npy", "bias.json"):
        shutil.copy(os.path.join(CACHE, f),
                    os.path.join(CACHE, "%s.pre_abort_%s" % (f, stamp)))
    st = get("/api/state")
    b0 = list(st["reloc"]["bias"])
    print("起点 位姿 %s  bias %s  吻合 %s  电压 %.2f V  busy %r  （快照 pre_abort_%s）"
          % (st["pose"], b0, None if st["reloc"]["agree"] is None else round(st["reloc"]["agree"], 3),
             st["volts"] or -1, st["busy"], stamp))

    direction = sys.argv[1] if len(sys.argv) > 1 else "forward"
    # Keep issuing the same short step until the detector pulls the motors. One command is not
    # enough, and the reason is worth stating because it is a property of the gate, not of the
    # car: MOTION_BAD wants two suspect samples 0.15 m apart, and `_closed_loop` crawls at its
    # 0.06 m/s floor over the last centimetres -- measured, a single 0.5 m Move ended at 1/2.
    # `motion.bad` is not cleared between commands, so the second one finishes the job.
    raw0, st, reason = None, get("/api/state"), None
    for k in range(4):
        p = move(0.5, direction)
        if raw0 is None:
            time.sleep(0.5)                   # rolling by then, so this lands mid-move
            raw0 = get("/api/state")["pose_raw"]
            yaw = math.radians(raw0[2])
            perp = (math.sin(yaw), -math.cos(yaw))
            off = [round(DIST * perp[0], 3), round(DIST * perp[1], 3)]
            rl = post("/api/reloc", {"dx": off[0], "dy": off[1]})
            print("已注入侧向偏移 %s m -> bias %s（车没动，只有位姿在说谎）" % (off, rl["bias"]))
            n0 = get("/api/state")["motion"]["n"]
            raw0 = st["pose_raw"]             # and this was the marker for "already injected"
        else:
            print("还没停，再来一条 0.5 m（第 %d 条）" % (k + 1))
        end = time.time() + 30
        while time.time() < end:
            st = get("/api/state")
            if st["motion"]["n"] > n0:
                break
            if p.poll() is not None and st["busy"] == "":
                break
            time.sleep(0.2)
        if st["motion"]["n"] > n0:
            reason = st["motion"]["why"]
            break
        out = p.communicate(timeout=20)[0].decode("utf-8", "replace")
        print("  第 %d 条 Move：%s  连续可疑 %d/2"
              % (k + 1, " ".join(out.split())[:150], st["motion"]["bad"]))
    raw0 = raw0 or st["pose_raw"]
    print("中止判定：%s" % (reason or "没有触发（motion.n 没涨）"))

    s = st
    m = s["motion"]
    print("行中检验最后一次采样：吻合 %.0f%% 峰值 %.0f%% @ (%+.2f,%+.2f) 未测 %.0f%% 连续可疑 %d 次，"
          "已中止 %d 次（RELOC_FLOOR 0.85）"
          % (100 * (m["zero"] or 0), 100 * (m["best"] or 0), m["dx"] or 0, m["dy"] or 0,
             100 * (m["outside"] or 0), m["bad"], m["n"]))
    print("里程计朝向：注入前 %s°，现在 %s°（车自己在倒车里转了就是打分糊掉的原因）"
          % (round(raw0[2], 1), round(s["pose_raw"][2], 1)))
    log = get("/api/log")
    rows = log if isinstance(log, list) else (log.get("rows") or log.get("log") or [])
    for row in rows[-6:]:
        print("  日志 %s" % json.dumps(row, ensure_ascii=False)[:200])

    print("\n等机器自己把位置找回来（停下后的静止检验，最多 25 s）")
    end = time.time() + 25
    while time.time() < end:
        s = get("/api/state")
        rl = s["reloc"]
        if rl["agree"] is not None and rl["agree"] > 0.9 and not rl["frozen"]:
            print("  恢复：吻合 %.0f%%  bias %s（原来 %s）  冻结 %s  建图 %s"
                  % (100 * rl["agree"], rl["bias"], b0, rl["frozen"], s["mapping"]))
            break
        time.sleep(1.0)
    else:
        print("  没自动恢复，bias %s  agree %s  frozen %s"
              % (get("/api/state")["reloc"]["bias"], get("/api/state")["reloc"]["agree"],
                 get("/api/state")["reloc"]["frozen"]))
    print("\n判定：%s" % ("行进中检查真的停了车" if reason else "没停——第 4 步没接上"))


if __name__ == "__main__":
    main()
