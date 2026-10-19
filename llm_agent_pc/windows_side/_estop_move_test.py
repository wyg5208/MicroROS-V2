"""急停能否打断一条正在跑的 MCP Move —— 真车 A/B。
usage: python -X utf8 _estop_move_test.py [stop|full] [distance]
stop 走聊天页面那颗按钮的同一个端点（:8020/stop），不是绕过界面直接戳控制台。
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
CHAT = "http://127.0.0.1:8020"
TRIGGER = 0.10          # 车真的走了 10 厘米以后再按停，确保是"行进中"


def pose():
    d = json.load(urllib.request.urlopen(CHAT + "/state", timeout=4))
    return d["car"]["pose"]


def estop():
    req = urllib.request.Request(CHAT + "/stop", data=b"{}", method="POST")
    return json.loads(urllib.request.urlopen(req, timeout=6).read().decode("utf-8"))


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "stop"
    dist = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    p0 = pose()
    print("起点      [%.3f, %.3f, %.1f]" % tuple(p0))

    mcp = McpClient(MCP, timeout=90.0)
    mcp.connect()
    box = {}

    def run():
        t0 = time.monotonic()
        try:
            box["text"] = mcp.call_tool("Move", {"direction": "forward", "distance": dist})
        except Exception as exc:
            box["err"] = "%s: %s" % (type(exc).__name__, exc)
        box["secs"] = time.monotonic() - t0

    th = threading.Thread(target=run)
    th.start()

    p_stop = None
    if mode == "stop":
        end_wait = time.monotonic() + 12.0
        while time.monotonic() < end_wait and th.is_alive():
            p = pose()
            if math.hypot(p[0] - p0[0], p[1] - p0[1]) >= TRIGGER:
                p_stop = p
                break
            time.sleep(0.05)
        if p_stop is None:
            print("!! 车没走到 %.2f m 就结束了，这一轮不算" % TRIGGER)
        else:
            print("按下急停  [%.3f, %.3f, %.1f]  已走 %.3f m"
                  % (p_stop[0], p_stop[1], p_stop[2],
                     math.hypot(p_stop[0] - p0[0], p_stop[1] - p0[1])))
            print("回包      %s" % estop())

    th.join(timeout=90)
    p1 = pose()
    gone = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    after = None if p_stop is None else math.hypot(p1[0] - p_stop[0], p1[1] - p_stop[1])
    print("Move 返回 %s  (%.2f 秒)" % (json.dumps(box, ensure_ascii=False), box.get("secs", 0)))
    print("终点      [%.3f, %.3f, %.1f]  总位移 %.3f / 命令 %.2f m" % (p1[0], p1[1], p1[2], gone, dist))
    if p_stop is not None:
        print("停后滑移  %.3f m（按下急停到最终停住之间多走的部分）" % after)


main()
