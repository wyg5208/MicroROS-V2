# -*- coding: utf-8 -*-
"""单次动作上限的离线判据：模型说的数、车上真执行的数、模型听到的解释，三者必须对得上。

为什么单独一份：`agent.py --self-test` 用的是 mock 大模型，它只会老实说 0.3 米 —— 截断那条分支
一辈子走不到。而"逐轮上限"（定时任务用）最容易出的事故恰恰是**压了却没人知道**：车走了 0.3，
模型汇报"我走了 1.2"，人和模型看的都不是同一串数。所以这里把大模型换成脚本好的、把 MCP 换成
记账的假对象，中间的循环是真的（run_turn / clamp_motion / 工具结果都走原路）。

跑法：C:\\Python314\\python.exe -X utf8 C:\\microros-agent\\pc_side\\_clamp_test.py
不连车、不连 :8000、不连大模型，全程零动作。

八组判据：超上限真被截 / 截断必须回灌给模型 / 逐轮压小生效 / 抬不动天花板 /
负方向对称 / 没超不许瞎报 / 只有 Move 受这对数管（边界要说清）/ 上限为 0 时不许放大。
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
    """只干一件事：把真的下发到车上那份参数记下来。"""

    def __init__(self):
        self.calls = []

    def call_tool(self, name, args):
        self.calls.append((name, dict(args)))
        return '{"execution_result": "success", "reason": "假执行（没碰车）"}'


class Recorder(object):
    """代替 ConsoleHooks / WebHooks：把事件收进列表，好看看页面上那一行到底写的什么数。"""

    def __init__(self):
        self.events = []

    def event(self, ev):
        self.events.append(ev)

    def confirm(self, name, args):
        raise AssertionError("这轮不该问人：cfg.confirm 一直是 False")


def one_turn(cap_dist, cap_ang, script):
    """跑真 run_turn：script 是模型按步要说的话（含 tool_calls）。返回 (MCP 收到的调用, 工具回包, 事件)。"""
    rec = Recorder()
    cfg = SimpleNamespace(model="假", llm_base="http://127.0.0.1:1/v1", api_key="x",
                          temperature=0.0, max_steps=len(script) + 2, dry_run=False,
                          vision_model=None, confirm=False, hooks=rec,
                          max_distance=cap_dist, max_angle=cap_ang)
    calls = iter(script)

    def fake_chat(_cfg, _messages, _tools=None, model=None):
        return dict(next(calls))

    mcp = FakeMcp()
    spec, name_map = agent.mcp_tools_to_openai([
        {"name": n, "description": n, "inputSchema": {"type": "object", "properties": {}}}
        for n in ("Move", "Tour")])
    real_chat, agent.openai_chat = agent.openai_chat, fake_chat
    try:
        messages = [{"role": "system", "content": agent.system_prompt(cfg)}]
        agent.run_turn(cfg, mcp, spec, name_map, messages, "指令")
    finally:
        agent.openai_chat = real_chat
    tools = [m["content"] for m in messages if m.get("role") == "tool"]
    return mcp.calls, tools, rec.events


def move_msg(distance=None, angle=None):
    """模型那一步的原始回复（一条 assistant 消息，不是 tool_calls 列表 —— 曾经搞反过一次）。"""
    args = {"direction": "forward" if (angle is None or distance is not None) else "turn_left"}
    if distance is not None:
        args["distance"] = distance
    if angle is not None:
        args["angle"] = angle
    return {"content": "我让车动一下。",
            "tool_calls": [{"id": "c1", "type": "function", "function": {
                "name": "Move", "arguments": json.dumps(args)}}]}


SCRIPT_DONE = [{"content": "做完了"}]

# 1 超限真被截：模型张口 5 米，车上最多收到 1.5
calls, tools, evs = one_turn(1.5, 90.0, [move_msg(distance=5.0)] + SCRIPT_DONE)
ck("模型说 5.0 米，MCP 收到的是 1.5", [calls[0][0], calls[0][1]["distance"]], ["Move", 1.5])
ck("  页面上那行「调用 Move {...}」写的也是截断后的数（人看的和车执行的必须同一份）",
   [e for e in evs if e["type"] == "call"][0]["args"]["distance"], 1.5)

# 2 截断要回灌给模型，不然它会按 5.0 汇报
ck("工具回包里有「被截断」这句和原值", ["截断" in tools[0], "5.0" in tools[0], "1.5" in tools[0]],
   [True, True, True])
ck("  还带上本轮上限是多少（逐轮上限可能比全局小，模型得知道按哪个数规划）",
   "本轮上限 1.5 米 / 90.0 度" in tools[0], True)

# 3 逐轮压小：start_turn 把 cfg 那对数改小之后，同一条 5.0 米只能走 0.3
calls, tools, evs = one_turn(0.3, 45.0, [move_msg(distance=5.0)] + SCRIPT_DONE)
ck("逐轮上限 0.3 生效（这就是任务表 max_distance 走的那条路）", calls[0][1]["distance"], 0.3)
ck("  回包说的是本轮的 0.3，不是全局那个数", "本轮上限 0.3 米 / 45.0 度" in tools[0], True)

# 4 抬不动天花板：逐轮只能往下压，这是 car_chat.py 那边定的规矩，这里复核 clamp 也照 cfg 走
calls, _, _ = one_turn(0.5, 90.0, [move_msg(distance=2.0)] + SCRIPT_DONE)
ck("cfg 比模型要的小的时候按 cfg（不是按 --max-distance 启动值放大回去）",
   calls[0][1]["distance"], 0.5)

# 5 负方向对称：倒车和反向转一样有边
calls, _, _ = one_turn(1.5, 90.0, [move_msg(distance=-2.0)] + SCRIPT_DONE)
ck("负方向夹到 -1.5，不是夹到 0", calls[0][1]["distance"], -1.5)
calls, _, _ = one_turn(1.5, 90.0, [move_msg(angle=-200.0, distance=None)] + SCRIPT_DONE)
ck("角度同样对称", calls[0][1]["angle"], -90.0)

# 6 没超就不许瞎报：这条判据要是永远为真，等于什么都没说
calls, tools, evs = one_turn(1.5, 90.0, [move_msg(distance=0.4, angle=0.0)] + SCRIPT_DONE)
ck("没超限：一个字不改，也不给模型加一句多余的「被截断」",
   [calls[0][1]["distance"], "截断" in tools[0]], [0.4, False])

# 7 边界要说清：这对数只管 Move。Tour/MapSpin/Unstick/RateTest 从来不受它约束，
#    逐轮上限也一样管不到它们 —— 别以为设了 max_distance 就把整班车关进笼子。
calls, tools, evs = one_turn(0.3, 45.0, [{"content": "我走一圈。", "tool_calls": [
    {"id": "t1", "type": "function", "function": {
        "name": "Tour", "arguments": json.dumps({"seconds": 120})}}]}] + SCRIPT_DONE)
ck("Tour 的秒数原样下发（上限只管 Move 的距离和角度）",
   [calls[0][0], calls[0][1]["seconds"], "截断" in tools[0]], ["Tour", 120, False])

# 8 上限压到 0 = 这一班什么都别动：不许把 0 当"没设"而放回大数
calls, tools, evs = one_turn(0.0, 0.0, [move_msg(distance=0.2)] + SCRIPT_DONE)
ck("max_distance=0 时距离被压成 0（0 是个数，不是缺省）", calls[0][1]["distance"], 0)
ck("  而且照样告诉模型被压了（不然它以为车挪了 0.2 米）", "截断" in tools[0], True)

print("\n%s" % ("CLAMP-TEST PASS" if not fails else "CLAMP-TEST FAIL：%s" % fails))
sys.exit(1 if fails else 0)
