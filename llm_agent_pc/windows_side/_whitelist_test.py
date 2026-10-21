# -*- coding: utf-8 -*-
"""逐轮工具白名单的离线判据：拦下来的工具必须**一次都没到车上**，而且模型要听得见"没执行"。

为什么单独一份：`_clamp_test.py` 管的是"这一步别走太远"，这份管的是"这一班根本不该动车"。
两者最容易出的事故形状不同 —— 白名单要是只在提示词里说一句"请不要调用 Tour"，小模型照样会调，
而人只在页面上看到一行"调用 Tour"就以为动作开始了；反过来要是拦得太狠（连 `Stop` 都拦、
或者拦完把整轮打死），只读班就再也说不清"我其实需要什么"。
所以这里把大模型换成脚本好的、把 MCP 换成记账的假对象，中间的循环是真的（run_turn / tool_gate /
clamp_motion / 工具回包都走原路）。

跑法：C:\\Python314\\python.exe -X utf8 C:\\microros-agent\\pc_side\\_whitelist_test.py
不连车、不连 :8000、不连大模型，全程零动作。
"""
import json
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import agent

fails = []


def ck(label, got, want):
    ok = got == want
    print(("OK  " if ok else "FAIL ") + label)
    if not ok:
        print("      got=%r want=%r" % (got, want))
        fails.append(label)


class FakeMcp(object):
    """只干一件事：把真的下发到车上那些调用记下来。白名单拦住的绝不该出现在这里。"""

    def __init__(self):
        self.calls = []

    def call_tool(self, name, args):
        self.calls.append((name, dict(args)))
        return '{"execution_result": "success", "reason": "假执行（没碰车）"}'


class Recorder(object):
    """代替 ConsoleHooks / WebHooks：页面/命令行看到的就是这份事件列表。"""

    def __init__(self):
        self.events = []

    def event(self, ev):
        self.events.append(ev)

    def confirm(self, name, args):
        raise AssertionError("这轮不该问人：cfg.confirm 一直是 False")


TOOLS = ("Move", "Tour", "Stop", "MapInfo", "GetStatus", "Beep")


def one_turn(script, allow=None, cap_dist=1.5, cap_ang=90.0):
    """跑真 run_turn。返回 (MCP 收到的调用, 工具回包, 事件)。"""
    rec = Recorder()
    cfg = SimpleNamespace(model="假", llm_base="http://127.0.0.1:1/v1", api_key="x",
                          temperature=0.0, max_steps=len(script) + 2, dry_run=False,
                          vision_model=None, confirm=False, hooks=rec,
                          max_distance=cap_dist, max_angle=cap_ang, allow_tools=allow)
    calls = iter(script)

    def fake_chat(_cfg, _messages, _tools=None, model=None):
        return dict(next(calls))

    mcp = FakeMcp()
    spec, name_map = agent.mcp_tools_to_openai([
        {"name": n, "description": n, "inputSchema": {"type": "object", "properties": {}}}
        for n in TOOLS])
    real_chat, agent.openai_chat = agent.openai_chat, fake_chat
    try:
        messages = [{"role": "system", "content": agent.system_prompt(cfg)}]
        agent.run_turn(cfg, mcp, spec, name_map, messages, "指令")
    finally:
        agent.openai_chat = real_chat
    tools = [m["content"] for m in messages if m.get("role") == "tool"]
    return mcp.calls, tools, rec.events


def ask(name, **args):
    """模型某一步的原始回复：一条 assistant 消息（不是 tool_calls 列表 —— 曾经搞反过一次）。"""
    return {"content": "我来一下。",
            "tool_calls": [{"id": "c_" + name, "type": "function", "function": {
                "name": name, "arguments": json.dumps(args)}}]}


DONE = [{"content": "做完了"}]


def kinds(evs):
    return [e["type"] for e in evs]


# 1 拦下来 = 一次都没到车上
calls, tools, evs = one_turn([ask("Move", distance=0.2)] + DONE, allow=["MapInfo"])
ck("白名单只有 MapInfo 时，模型调 Move 一次都没下发到车", calls, [])
ck("  页面上不许出现「调用 Move」那一行（出现了人就以为动作开始了）",
   [e for e in evs if e["type"] == "call"], [])
ck("  但要有一条 blocked 事件，带工具名和理由（渲染这一条的就是人看到的那句）",
   [[e["tool"], "白名单" in e["text"]] for e in evs if e["type"] == "blocked"], [["Move", True]])
ck("  回包明说这个工具没执行、车上什么都没发生",
   ["没有执行" in tools[0], "只允许：MapInfo" in tools[0]], [True, True])
ck("  回包里绝不含假执行那句 success（含上了就等于谎称动过）", "success" in tools[0], False)

# 2 白名单里的照常走
calls, tools, evs = one_turn([ask("MapInfo")] + DONE, allow=["MapInfo"])
ck("白名单里的工具照常到车（拦的是名单外的，不是这一轮全都别动）",
   [calls, [e["type"] for e in evs if e["type"] in ("call", "blocked")]],
   [[("MapInfo", {})], ["call"]])

# 3 Stop 永远放行：拦掉它拦不住任何动作，只会让模型想收手都收不了
calls, tools, evs = one_turn([ask("Stop")] + DONE, allow=["MapInfo"])
ck("Stop 不在白名单里也照样执行", calls, [("Stop", {})])
ck("  而 Tour 仍在门外（放行的是那一个，不是整个动作类）",
   bool(agent.tool_gate(SimpleNamespace(allow_tools=["MapInfo"]), "Tour")), True)

# 4 同一批里拦一个，不许顺手把同批其余的也丢掉
calls, tools, evs = one_turn([{"content": "两个一起。", "tool_calls": [
    {"id": "a", "type": "function", "function": {"name": "Tour", "arguments": "{}"}},
    {"id": "b", "type": "function", "function": {"name": "GetStatus", "arguments": "{}"}}]}] + DONE,
    allow=["GetStatus"])
ck("一条 Move 被拦不等于这一步全废：同批的 GetStatus 仍然执行",
   [calls, len([e for e in evs if e["type"] == "blocked"])], [[("GetStatus", {})], 1])

# 5 没设白名单 = 行为和从前一字不差（这条是整个改动的"没回归"证明）
calls, tools, evs = one_turn([ask("Move", distance=0.2)] + DONE, allow=None)
ck("allow_tools=None 时不限任何工具、不多一句解释",
   [calls[0][0], any("白名单" in t for t in tools)], ["Move", False])

# 6 两道闸共存：名单里有 Move，超限照样截断
calls, tools, evs = one_turn([ask("Move", distance=5.0)] + DONE, allow=["Move"])
ck("白名单放开 Move 之后，单次上限仍然管着它（两道闸不打架、不互相顶掉）",
   [calls[0][1]["distance"], "截断" in tools[0], "白名单" in tools[0]], [1.5, True, False])

# 7 空列表是"一个都不许"，和 None 不是一回事
calls, tools, evs = one_turn([ask("GetStatus")] + DONE, allow=[])
ck("allow_tools=[] 是数出来的空集（什么都别调），不是「没写」", calls, [])
ck("  拒绝文本仍然给模型看（它要能说出「我需要放开哪一个」）",
   "只允许：" in tools[0], True)

# 8 拦一次不等于这轮死了：模型换用白名单里的工具还能把活干完
calls, tools, evs = one_turn([ask("Beep", duration_ms=200), ask("MapInfo")] + DONE,
                             allow=["MapInfo"])
ck("被拦之后仍能接着用白名单里的工具（不 break、不空转到 max_steps）",
   [calls, [t for t in tools if "success" in t]], [[("MapInfo", {})], ['{"execution_result": "success", "reason": "假执行（没碰车）"}']])

print("\n%s" % ("WHITELIST-TEST PASS" if not fails else "WHITELIST-TEST FAIL：%s" % fails))
sys.exit(1 if fails else 0)
