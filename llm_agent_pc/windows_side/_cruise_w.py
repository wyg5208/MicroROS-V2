# -*- coding: utf-8 -*-
"""直行巡航时底盘到底听不听 cmd_vel.angular.z？

第 21 任务当时判"不听"，但那组数据现在被污染了：控制台空闲发布线程每 0.2 s 往
/cmd_vel 塞一次 (0,0)，而 MCP Move 不 claim busy，所以"巡航 + 转向指令"那几秒里
车其实一直在被点刹。本轮走 /api/teleop（活跃期间发布线程不会插零），四条腿的净
位移和净朝向都回到 ~0，最大单边行程 0.25 m，插着电源线也缠不到线。

判读：w 真被执行 -> 每腿 yaw 变化约 +-11°（0.3 rad/s 命令 ~ 9°/s）；
      w 被吞   -> 四条腿的 yaw 变化都贴在 0 上，而 x/y 该走多少走多少。
"""
import json
import sys
import time
import urllib.request

CONSOLE = "http://127.0.0.1:8010"
V, W, T = 0.20, 0.30, 1.2


def get(path):
    return json.loads(urllib.request.urlopen(CONSOLE + path, timeout=3).read())


def post(path, obj):
    req = urllib.request.Request(CONSOLE + path, data=json.dumps(obj).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=2).read()
    except Exception:                                          # noqa: BLE001
        pass


def leg(v, w, label):
    t0 = time.monotonic()
    p0 = get("/api/state")["pose"]
    rows = []
    while time.monotonic() - t0 < T:
        post("/api/teleop", {"v": v, "w": w, "hold": 0.3})
        s = get("/api/state")
        rows.append((time.monotonic(), s["pose"], s["teleop"]["v"], s["teleop"]["w"]))
        time.sleep(0.09)
    post("/api/teleop", {"v": 0.0, "w": 0.0, "hold": 0.3})
    time.sleep(1.2)                       # 等位姿停稳再读落点
    p1 = get("/api/state")["pose"]
    dyaw = (p1[2] - p0[2] + 180.0) % 360.0 - 180.0
    moved = sum(((rows[i][1][0] - rows[i - 1][1][0]) ** 2 +
                 (rows[i][1][1] - rows[i - 1][1][1]) ** 2) ** 0.5
                for i in range(1, len(rows)))
    ok = [r for r in rows if r[2] != 0.0 or r[3] != 0.0]
    print("%-8s 指令 v=%+.2f w=%+.2f  采样 %d（%d 帧看到非零）  "
          "净偏航 %+6.2f°  行程 %.2f m  峰值 |w| 读数 %.2f" % (
              label, v, w, len(rows), len(ok), dyaw, moved,
              max((abs(r[3]) for r in rows), default=0.0)))
    return dyaw


st = get("/api/state")
print("起步 %s  busy=%r  建图=%s  电压 %.2f V" %
      (tuple(round(v, 2) for v in st["pose"]), st["busy"], st["mapping"], st["volts"]))
sys.stdout.flush()
if st["busy"]:
    print("电机已经被占用，取消")
    raise SystemExit
if st["mapping"]:
    # 这几条腿是故意歪着走的：净朝向回 0，但每一腿中间的假位姿会把墙折成两道
    print("建图还开着：POST /api/mapping false 之后再来，测完记得恢复")
    raise SystemExit

tot = 0.0
for label, v, w in (("左A", V, W), ("左B", -V, W), ("右A", V, -W), ("右B", -V, -W)):
    tot += leg(v, w, label)
    sys.stdout.flush()
post("/api/teleop", {"v": 0.0, "w": 0.0, "hold": 0.5})
fin = get("/api/state")
print("四条腿净偏航合计 %+6.2f°（期望 ~0）  结束位姿 %s  吻合=%s  冻结=%s" % (
    tot, tuple(round(v, 2) for v in fin["pose"]),
    round(fin["reloc"]["agree"], 3) if fin["reloc"]["agree"] is not None else None,
    fin["reloc"]["frozen"]))
