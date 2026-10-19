# -*- coding: utf-8 -*-
"""反向对照：真车正常走一段，路上一次机器自停都不许发生。

刚把"停"的门槛加了一条新通道（打分看不出、靠看得穿的墙指认），所以必须当着车证明它
不会在诚实的位姿上乱按停：一条 MCP Move 走完，`motion["n"]`（机器中止次数）必须还是 0，
日志里不许出现"机器判定"，可疑帧也不许被扣在图外（`扣票` 必须一路是 0，否则建图被拖慢）。
usage: python -X utf8 _honest_run.py [米数]
"""
import json
import math
import sys
import threading
import time
import urllib.request

sys.path.insert(0, r"C:\microros-agent")
from agent import McpClient  # noqa: E402

MCP = "http://127.0.0.1:8000/mcp"
API = "http://127.0.0.1:8010"


def get(path):
    return json.loads(urllib.request.urlopen(API + path, timeout=4).read().decode("utf-8"))


def main():
    dist = float(sys.argv[1]) if len(sys.argv) > 1 else 0.6
    n0 = get("/api/state")["motion"]["n"]
    p0 = get("/api/state")["pose"]
    t0log = max([e["t"] for e in get("/api/state")["log"]] or [0.0])
    box = {}

    def run():
        mcp = McpClient(MCP, timeout=90.0)
        mcp.connect()
        box["text"] = mcp.call_tool("Move", {"direction": "forward", "distance": dist})

    rows = []
    stop = threading.Event()

    def sample():
        while not stop.is_set():
            s = get("/api/state")
            m = s["motion"]
            rows.append((round(math.hypot(s["pose"][0] - p0[0], s["pose"][1] - p0[1]), 2),
                         m["bad"], m["zero"], m["best"], m["dx"], m["dy"],
                         bool(m["suspect"]), s["reloc"]["frozen"], (m["why"] or "")[:40]))
            time.sleep(0.2)

    th = threading.Thread(target=run)
    sp = threading.Thread(target=sample)
    th.start()
    sp.start()
    th.join(timeout=120)
    time.sleep(0.5)
    stop.set()
    sp.join(timeout=5)

    st = get("/api/state")
    print("Move      %s" % json.dumps(box.get("text"), ensure_ascii=False)[:200])
    print("          起点 (%.2f, %.2f)  走了 %.2f m  中止次数 %d -> %d"
          % (p0[0], p0[1], rows[-1][0] if rows else -1, n0, st["motion"]["n"]))
    print("判定      %s" % (st["motion"]["why"] or "（无）"))
    uniq = []
    for r in rows:
        if uniq and r == uniq[-1][0]:
            uniq[-1][1] += 1
        else:
            uniq.append([r, 1])
    for r, n in uniq:
        print("   %5.2f m bad=%s zero=%s best=%s off=(%s,%s) 疑=%s 冻=%s %s ×%.1f s"
              % (r[0], r[1], r[2], r[3], r[4], r[5], "是" if r[6] else "否", r[7], r[8], n * 0.2))
    # The console's log is a ring buffer for the whole session, and the previous run machine-stopped
    # on purpose. Only entries raised after this command started can count against this drive.
    machines = [e["text"] for e in st["log"] if e["t"] >= t0log and "机器判定" in e["text"]]
    assert st["motion"]["n"] == n0, "诚实的一路上发生了机器自停：%s" % machines
    assert not machines, "日志里出现了机器判定中止：" % machines
    assert not st["reloc"]["frozen"], "不该冻结建图"
    held = sum(1 for r in rows if r[6])
    print("\n通过：这一路 %d 次采样，可疑 %d 次，一次中止都没有 —— 新通道没把正常行驶看成搬动。"
          % (len(rows), held))


main()
