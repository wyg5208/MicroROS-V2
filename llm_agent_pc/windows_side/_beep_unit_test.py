# -*- coding: utf-8 -*-
"""定案 /beep 的单位：厂商两份文档互相矛盾（一份说 data 是毫秒，一份说是 10 毫秒的节拍数）。

跑法（车通电、bridge 在跑的时候）：
    _mcp_run.bat _beep_unit_test.py
或
    python -X utf8 _beep_unit_test.py

每档各响一次，人耳掐表把秒数填进去。两档就够定性：如果单位是毫秒，300 响 0.3 秒、
3000 响 3 秒；如果是 10 ms 节拍，300 响 3 秒、3000 响 30 秒（会响到让人怀疑人生）。
答案写进 _beep_unit.jsonl，定案之后才改 Beep 工具里的语义。

不需要新依赖：走 bridge 自己的 :8000 MCP 端点，和 _mcp_call.py 同一套。
"""
import json
import sys
import time

from _mcp_call import rpc, _body

STEPS = [(300, "预期：毫秒=0.3 s，10ms 节拍=3 s"),
         (1000, "预期：毫秒=1 s，10ms 节拍=10 s"),
         (3000, "预期：毫秒=3 s，10ms 节拍=30 s（这一档最能区分）")]


def beep(ms):
    """One Beep call on the live bridge, session included."""
    _t, sid = rpc({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                   "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                              "clientInfo": {"name": "beep-unit-probe", "version": "1"}}})
    out, _ = rpc({"jsonrpc": "2.0", "id": 2, "method": "tools/call",
                  "params": {"name": "Beep", "arguments": {"duration_ms": ms}}}, sid)
    body = _body(out)
    text = "".join(c.get("text", "") for c in body.get("result", {}).get("content", []))
    return text or json.dumps(body, ensure_ascii=False)[:200]


def main():
    only = [int(a) for a in sys.argv[1:] if a.isdigit()]
    steps = [s for s in STEPS if not only or s[0] in only]
    print("/beep 单位定案：旁边有人听着，响完立刻按回车并填掐到的秒数（直接回车=跳过）\n")
    rows = []
    for ms, expect in steps:
        print("即将发布 /beep data=%d —— %s" % (ms, expect))
        input("按下回车让它响 ... ")
        t0 = time.time()
        print("  已发布：%s" % beep(ms))
        try:
            heard = input("  实际响了大约几秒？（回车跳过）: ").strip()
        except (EOFError, KeyboardInterrupt):
            heard = ""
        e = {"sent": ms, "heard_s": float(heard) if heard else None,
             "rpc_rtt_s": round(time.time() - t0, 2), "t": time.strftime("%F %T")}
        rows.append(e)
        print("  记录：%r\n" % e)
    with open("_beep_unit.jsonl", "a", encoding="utf-8") as f:
        for e in rows:
            f.write(json.dumps(e, ensure_ascii=False) + "\n")
    print("已写入 _beep_unit.jsonl。判读：heard_s ≈ sent/1000 → 单位是毫秒；"
          "heard_s ≈ sent/100 → 单位是 10 ms 节拍。")


if __name__ == "__main__":
    main()
