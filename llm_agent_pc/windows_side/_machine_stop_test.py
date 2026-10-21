"""机器自停走的是它自己的判词，不是"收到停止指令" —— 真车验证。
usage: python -X utf8 _machine_stop_test.py drives [dx] [几条命令] [是否停折叠] [先倒退米数] [命令米数] [dy]
在一条 MCP Move 跑到一半时，用 /api/reloc 故意把位姿平移 offset 米（等价于车被搬走一段，
只是不用手）。运动中检测器连判两次就该带着自己的判词停车；回包里若出现"收到停止指令"
说明人工/机器两条通道又混了。结束后把 bias 与建图状态还原。
"""
import json
import math
import sys
import threading
import time
import urllib.request

from drives_gate import require_drives  # noqa: E402

require_drives("跑一条 0.8 m 的 Move，中途注入位姿平移逼机器自停，事后还原它自己读到的 bias 与建图")

sys.path.insert(0, r"C:\microros-agent")
from agent import McpClient  # noqa: E402

MCP = "http://127.0.0.1:8000/mcp"
API = "http://127.0.0.1:8010"
DIST = 0.8              # 命令的距离：要长到能容得下两次判定
TRIGGER = 0.12          # 真的走出一段再注入，确保是"行进中"


def get(path):
    return json.loads(urllib.request.urlopen(API + path, timeout=4).read().decode("utf-8"))


def post(path, body):
    req = urllib.request.Request(API + path, data=json.dumps(body).encode("utf-8"),
                                 method="POST",
                                 headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=8).read().decode("utf-8"))


def main():
    off = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
    reps = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    nomap = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    back = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    # argv[5]: how far to command. It is an argument because of how the lie is injected:
    # `_closed_loop` counts progress as *net displacement in the corrected pose*, so the teleport
    # itself books off `offset` metres and a 0.8 m command ends the instant the lie lands -- the
    # car then travels no more ground, the rolling detector never gets its second sample, and the
    # run that "failed" was a test artifact, not a missed abort. A real carry consumes the same
    # budget, so a tour would hit this exactly the way it is reproduced here. Command
    # offset + trigger + 0.4 m so two MOTION_ROLL samples still fit after the jump.
    dist = float(sys.argv[5]) if len(sys.argv) > 5 else DIST
    # argv[6]: the y half of the lie, in the odometry frame. It is an argument because the map gets
    # *used up* as a test subject: every injection folds a fiction room into the grid, and a second
    # identical injection is then perfectly consistent with the first -- measured live, the same
    # +0.8 m x jump read 吻合 0.859 the second time (margin 0.119, no see-through contrast) after
    # 0.671 the first, i.e. nothing local can tell it from an honest pose any more. Inject along a
    # vector the grid has not already been made to agree with.
    offy = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
    st = get("/api/state")
    bias0 = list(st["drift"]["bias"])
    p0 = st["pose"]
    print("起点      [%.3f, %.3f, %.1f]  bias (%.3f, %.3f)" % (p0[0], p0[1], p0[2], bias0[0], bias0[1]))

    mcp = McpClient(MCP, timeout=90.0)
    mcp.connect()
    box = {}
    if back > 0:
        # 上一轮把车顶到墙边了，倒回去一点再测：不移动的车注入不了"行进中"的偏差
        print("让位      %s" % mcp.call_tool("Move", {"direction": "backward", "distance": back}))
        p0 = get("/api/state")["pose"]   # 起点跟着挪，否则刚倒完就"满足"了 TRIGGER

    def run():
        t0 = time.monotonic()
        box["texts"] = []
        for i in range(reps):
            try:
                box["texts"].append(mcp.call_tool("Move", {"direction": "forward", "distance": dist}))
            except Exception as exc:
                box["texts"].append("%s: %s" % (type(exc).__name__, exc))
                break
            if get("/api/state")["reloc"]["frozen"]:
                break          # the machine stopped folding; keep driving would only smear the map
        box["secs"] = time.monotonic() - t0

    trace = []

    def sample():
        """每 0.1 s 抄一份"检测器此刻看到什么"。MOTION_BAD 要连续两样，光看结尾的 1/2 说不出
        是哪一帧被哪一道门退回去的。

        行进中那一半已经有数据了，缺的是**两条命令之间停住的那一秒**：折叠循环照常跑，
        `_hold_vote` 是不是还在拦、静止判定走到了哪一条，都只能从这里看。
        """
        while th.is_alive():
            s = get("/api/state")
            m = s["motion"] or {}
            rl = s["reloc"] or {}
            trace.append("%.2f m bad=%s zero=%s best=%s off=(%s,%s) 疑=%s 停=(%s/%s@%s) %s%s %s"
                         % (math.hypot(s["pose"][0] - p0[0], s["pose"][1] - p0[1]),
                            m.get("bad"), m.get("zero"), m.get("best"), m.get("dx"), m.get("dy"),
                            "是" if m.get("suspect") else "否",
                            rl.get("agree"), rl.get("best"), rl.get("dx"),
                            "图=%s" % ("停" if not s["mapping"] else "走"),
                            "冻" if rl.get("frozen") else "",
                            (rl.get("why") or m.get("why") or "")[:44]))
            time.sleep(0.1)

    th = threading.Thread(target=run)
    th.start()
    sp = threading.Thread(target=sample)
    sp.start()

    injected = None
    end_wait = time.monotonic() + 14.0
    while time.monotonic() < end_wait and th.is_alive():
        p = get("/api/state")["pose"]
        if math.hypot(p[0] - p0[0], p[1] - p0[1]) >= TRIGGER:
            injected = post("/api/reloc", {"dx": off, "dy": offy})
            if nomap:
                # 对照组：注入后立刻停掉建图折叠。真车实测里吻合度从 0.71 自己爬回 0.96 —— 谎被
                # 折进了图里，检测器的证据就这么没了。判不了是不是这个原因，就别判。
                post("/api/mapping", {"on": False})
            print("注入偏差  已把位姿平移 (%+.2f, %+.2f) m -> bias %s（mapping=%s）"
                  % (off, offy, injected["bias"], injected["mapping"]))
            break
        time.sleep(0.05)
    if injected is None:
        print("!! 没能在行进中注入，这一轮不算")

    th.join(timeout=90)
    st = get("/api/state")
    print("Move 返回 %s  (%.2f 秒)" % (json.dumps(box, ensure_ascii=False), box.get("secs", 0)))
    print("判定      %s" % (st["motion"] or {}).get("why"))
    print("过程      %d 次采样（相同行合并）" % len(trace))
    uniq = []
    for t in trace:
        if uniq and t == uniq[-1][0]:
            uniq[-1][1] += 1
        else:
            uniq.append([t, 1])
    for t, n in uniq:
        print("   %s  ×%.1f s" % (t, n * 0.1))
    print("横幅      frozen=%s mapping=%s why=%s"
          % (st["reloc"]["frozen"], st["mapping"], st["reloc"]["why"]))
    print("日志尾巴  %s" % " | ".join(e["text"][:90] for e in st["log"][:3]))

    now = list(st["drift"]["bias"])
    back = post("/api/reloc", {"dx": round(bias0[0] - now[0], 3), "dy": round(bias0[1] - now[1], 3)})
    post("/api/mapping", {"on": True})
    st = get("/api/state")
    print("还原      bias %s -> %s（原 %s）mapping=%s frozen=%s"
          % (now, back["bias"], bias0, st["mapping"], st["reloc"]["frozen"]))


main()
