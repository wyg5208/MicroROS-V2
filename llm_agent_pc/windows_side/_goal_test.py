# -*- coding: utf-8 -*-
"""长程目标的离线判据：目标只是「每轮重述的一句话」，它不许变成第二条动作通道。

为什么单独一份：两道逐轮闸管的是「这一步别走太远 / 这一句根本不许动车」，都只在一轮之内。
目标是跨轮的，最容易出的事故形状不一样：
  1. 悄悄变成自动指令 —— 所以判据盯着「没有目标时必须与从前一字不差」，注入的只是人那句话的追加；
  2. 判暗号判松了 —— 注入那句里本来就写着暗号这五个字，全文搜会把「我该写暗号了」当成做完，
     目标被抹掉是找不回来的（判漏了只是多烧几轮，有上限兜底），所以取严的那个并钉住它；
  3. 一环坏了永远挂着 —— 半路炸的那一轮也必须烧额度；
  4. 页面上写着两套暗号 —— 字样只从 :8020/state 的 chat.goal_sentinel 拿，页面抄一份就是两个真相。

:8020 那几条路由是在本机**随机端口**的测试实例里跑真 Handler（不开 8020，也不碰正在跑的那个），
大模型换成脚本好的、MCP 换成不会动的假对象，车一次都不碰。

跑法：C:/Python314/python.exe -X utf8 C:/microros-agent/pc_side/_goal_test.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import agent           # noqa: E402
import car_chat        # noqa: E402

fails = []
skips = []


def ck(label, got, want):
    ok = got == want
    print(("OK  " if ok else "FAIL ") + label)
    if not ok:
        print("      got=%r want=%r" % (got, want))
        fails.append(label)


TOOLS = ("Move", "Tour", "Stop", "MapInfo", "GetStatus", "Beep")
SPEC, NAME_MAP = agent.mcp_tools_to_openai([
    {"name": n, "description": n, "inputSchema": {"type": "object", "properties": {}}}
    for n in TOOLS])

SCRIPT = {"reply": "", "texts": [], "block": None, "boom": None}


def fake_run_turn(cfg, mcp, tools_spec, name_map, messages, user_text):
    """顶掉 agent.run_turn：把要发往模型的那句话留着，并脚本好这一轮的回答。"""
    SCRIPT["texts"].append(user_text)
    if SCRIPT["block"] is not None:
        SCRIPT["block"].wait(8.0)
    if SCRIPT["boom"] is not None:
        raise SCRIPT["boom"]
    cfg.hooks.event({"type": "assistant", "text": SCRIPT["reply"]})


agent.run_turn = fake_run_turn


class FakeMcp(object):
    def call_tool(self, name, args):
        raise AssertionError("这份判据不该有任何调用到车：%s" % name)


def new_cfg():
    return SimpleNamespace(model="假", llm_base="http://127.0.0.1:1/v1", api_key="x",
                           temperature=0.0, max_steps=4, dry_run=False, vision_model=None,
                           confirm=False, hooks=None, max_distance=1.5, max_angle=90.0,
                           allow_tools=None, mcp_url="http://127.0.0.1:1/mcp")


TMP = tempfile.mkdtemp(prefix="goal_test_")
_seq = [0]


def fresh(goal_file=None):
    """新会话 = 新的一堆事件；goal_file 指到同一个文件就等于「重启 :8020」。"""
    if goal_file is None:
        _seq[0] += 1
        car_chat.GOAL_FILE = os.path.join(TMP, "goal_%d.json" % _seq[0])
    else:
        car_chat.GOAL_FILE = goal_file
    return car_chat.Session(new_cfg(), FakeMcp(), SPEC, NAME_MAP, console="http://127.0.0.1:1")


def wait_idle(s, secs=6.0):
    t0 = time.time()
    while s.busy and time.time() - t0 < secs:
        time.sleep(0.005)
    return not s.busy


def turn(s, text="去看一眼", reply="这一步走了书房，还剩厨房没测。", **kw):
    """真走 start_turn（含注入、线程、记账），返回发往模型的那句话。"""
    SCRIPT["reply"] = reply
    n = len(SCRIPT["texts"])
    s.start_turn(text, **kw)
    if not wait_idle(s):
        return None
    return SCRIPT["texts"][n] if len(SCRIPT["texts"]) > n else None


def events(s, prefix=None):
    with s.lock:
        evs = list(s.ring)
    return [e for e in evs if prefix is None or e["type"].startswith(prefix)]


def goal_evs(s):
    return [(e["type"], e.get("turns_left"), e.get("why") or "") for e in events(s, "goal")]


def brief(d):
    """目标状态里只有这三样要跨重启对得上（set_at 是时间戳，不比）。"""
    if not d:
        return None
    return [d["text"], d["turns_left"], d["status"]]


# ============ A 没有目标时，与从前一字不差 ============
s = fresh()
ck("A1 新会话没有目标（goal 与 /state 那格都是 None，不是空字符串）",
   [s.goal, s.goal_payload()], [None, None])
ck("A2 没目标时发往模型的就是人那句话本身，一个字都没多加",
   turn(s, "去看一眼"), "去看一眼")
ck("A3  连「长程目标」和暗号都不出现（出现了等于每轮提一遍不存在的目标）",
   ("长程目标" in SCRIPT["texts"][-1]) or (car_chat.GOAL_SENTINEL in SCRIPT["texts"][-1]), False)
ck("A4 没目标的回合不产生任何 goal 事件（不然页面会凭空冒出一行）", goal_evs(s), [])
ck("A6 没设目标时两道闸照旧按天花板走（目标不参与它们的默认值）",
   [s.gates["max_distance"], s.gates["max_angle"], s.gates["allow_tools"]], [1.5, 90.0, None])
turn(s, "去看一眼", max_distance=0.5)
ck("A5 压了逐轮上限、没压目标时，追加的只有那一句上限（顺序也没变）",
   [SCRIPT["texts"][-1].startswith("去看一眼"),
    SCRIPT["texts"][-1].count("[本轮单次动作上限"), "长程目标" in SCRIPT["texts"][-1]],
   [True, 1, False])

# ============ B 注入：目标是「重述」，不是「替人说话」 ============
gf = os.path.join(TMP, "goal_B.json")
s = fresh(gf)
payload, err = s.set_goal("把客厅走完", 4)
ck("B1 set_goal 返回生效那一份（4 轮、active、没理由）",
   [err, brief(payload)], ["", ["把客厅走完", 4, "active"]])
base = turn(fresh(), "去看一眼")      # 同输入、无目标的那一份，用来逐字节比
cap = turn(s, "去看一眼")
ck("B2 带目标那句 = 无目标那句 + 末尾追加（人那句话前面没被塞东西）",
   [cap.startswith(base), len(cap) > len(base)], [True, True])
ck("B3  追加里写着还剩几轮、目标原文、和暗号字样",
   all(x in cap for x in ("[长程目标（还剩 4 轮）", "把客厅走完", car_chat.GOAL_SENTINEL)), True)
ck("B4 没写暗号就只是烧掉一轮（状态还是 active）",
   [s.goal_payload()["status"], s.goal_payload()["turns_left"]], ["active", 3])
ck("B5 事件流里有 goal_set（挂上那一刻）和 goal_tick（这一轮记过了）",
   [e[0] for e in goal_evs(s)], ["goal_set", "goal_tick"])
ck("B6  两条都带上剩几轮（页面不用再去别处拼）",
   [goal_evs(s)[0][1], goal_evs(s)[1][1]], [4, 3])
turn(s, "报一下电量")
ck("B7 随口问的一句也算一轮（目标是每轮重述，不是每轮干活）",
   s.goal_payload()["turns_left"], 2)

# ============ C 暗号只认单独占最后一行 ============
ck("C1 暗号自己占一行 = 宣称做完", car_chat.goal_done_says("都看完了\n[目标完成]"), True)
ck("C2  带 markdown 装饰也算（模型爱加粗）", car_chat.goal_done_says("好了\n**[目标完成]**"), True)
ck("C3 半句里提到暗号不算：注入那句就写着它，全文搜会把「该写了」当成「写完了」",
   car_chat.goal_done_says("我该写 [目标完成] 了\n但还差厨房"), False)
ck("C4 和别的话挤在同一行也不算 —— 宁可多烧几轮，不许把目标抹掉",
   car_chat.goal_done_says("还剩厨房。[目标完成]"), False)
ck("C5 空回答不算", car_chat.goal_done_says(""), False)
s = fresh()
s.set_goal("把客厅走完", 4)
turn(s, "去看一眼", reply="都看完了，也回到充电座了。\n[目标完成]")
ck("C6 判到暗号就 done、额度清零、发 goal_done",
   [brief(s.goal_payload()), goal_evs(s)[-1][0]], [["把客厅走完", 0, "done"], "goal_done"])
ck("C7  done 之后不再注入（它已经不是挂着的那句话了）",
   "长程目标" in turn(s, "再看一下书房"), False)
s = fresh()
s.set_goal("把客厅走完", 4)
turn(s, "去看一眼", reply="按规矩下一句我该写 [目标完成]\n但这一步只走到书房。")
ck("C8  只在正文里引用暗号 → 走的是「没做完」那条路",
   brief(s.goal_payload()), ["把客厅走完", 3, "active"])

# ============ D 轮数到头 / 半路炸了 ============
s = fresh()
s.set_goal("把客厅走完", 1)
turn(s, "去看一眼", reply="还在走。")
ck("D1 额度用到头自动作废（expired，不是再给它一次）",
   [brief(s.goal_payload()), goal_evs(s)[-1][0]], [["把客厅走完", 0, "expired"], "goal_expired"])
ck("D2  作废那句排在收尾之前（记账赶在放开 busy 前做完，见 car_chat 的 _run）",
   [e["type"] for e in events(s)][-2:], ["goal_expired", "turn_end"])
ck("D3 作废后下一句不再带上它", "长程目标" in turn(s, "随便问一句"), False)
s = fresh()
s.set_goal("把客厅走完", 2)
SCRIPT["boom"] = RuntimeError("假炸：模型连不上")
turn(s, "去看一眼")
SCRIPT["boom"] = None
ck("D4 半路炸的那一轮也烧了额度（不烧等于坏掉的一环永远挂着）",
   [s.goal_payload()["turns_left"], any(e["type"] == "error" for e in events(s))], [1, True])
s = fresh()
s.set_goal("把客厅走完", 1)
SCRIPT["boom"] = RuntimeError("假炸")
turn(s, "去看一眼")
SCRIPT["boom"] = None
ck("D5  炸在最后那一轮也照样判过期，不会卡在 active", s.goal_payload()["status"], "expired")

# ============ E 落盘：重启 :8020 不该忘掉目标 ============
gf = os.path.join(TMP, "goal_E.json")
s = fresh(gf)
s.set_goal("找到沙发并记住它", 6)
turn(s, "去看一眼", reply="还没看到。")
again = fresh(gf)
ck("E1 换个会话读同一份文件 = 恢复成上一轮记过账后的状态",
   brief(again.goal_payload()), ["找到沙发并记住它", 5, "active"])
ck("E2 恢复的会话下一句照样带上它（轮数是记过账的，不是原始的）",
   "[长程目标（还剩 5 轮）" in turn(again, "去看一眼"), True)
again.clear_goal("人不想要了")
ck("E3 清除写盘是 null（重启后不会诈尸），文件本身还在",
   [json.loads(io.open(gf, encoding="utf-8").read()), again.goal_payload()], [None, None])
ck("E4  清除发了 goal_cleared，并留着原来那句话（不然人不知道丢了什么）",
   goal_evs(again)[-1][0], "goal_cleared")
ck("E5 没有目标时清除是空操作（幂等：页面上连点两下不报错）",
   [again.clear_goal(), brief(again.goal_payload())], [False, None])
io.open(gf, "w", encoding="utf-8").write('{"text": "半截不')
ck("E6 文件坏了不炸启动，按没有目标处理", fresh(gf).goal_payload(), None)
io.open(gf, "w", encoding="utf-8").write(json.dumps(
    {"text": "太久远的目标", "turns_left": 999, "status": "active", "set_at": 1}))
ck("E7 文件里轮数越界要压回上限（重启不是绕过上限的门）",
   fresh(gf).goal_payload()["turns_left"], car_chat.GOAL_MAX_TURNS)
io.open(gf, "w", encoding="utf-8").write(json.dumps(
    {"text": "零轮还想跑", "turns_left": 0, "status": "active", "set_at": 1}))
ck("E8  active 但剩 0 轮是自相矛盾的存档 → 读成 expired",
   fresh(gf).goal_payload()["status"], "expired")
io.open(gf, "w", encoding="utf-8").write(json.dumps(
    {"text": "怪状态", "turns_left": 3, "status": "whatever", "set_at": 1}))
ck("E9  认不出的状态一律按 expired，绝不按 active（读盘不能把未知当续跑）",
   fresh(gf).goal_payload()["status"], "expired")
io.open(gf, "w", encoding="utf-8").write(json.dumps(
    {"text": "超" + "长" * 400, "turns_left": 3, "status": "active", "set_at": 1}))
ck("E10 存档里超长的文本读回来先截到上限",
   len(fresh(gf).goal_payload()["text"]) <= car_chat.GOAL_MAX_CHARS, True)
buf = io.StringIO()
car_chat.GOAL_FILE = os.path.join(TMP, "no_such_dir", "goal.json")
s = car_chat.Session(new_cfg(), FakeMcp(), SPEC, NAME_MAP, console="http://127.0.0.1:1")
with contextlib.redirect_stderr(buf):
    p, e = s.set_goal("写不进去的目标", 3)
ck("E11 落盘失败不影响这一轮（目标照样生效），但必须留一行日志",
   [e == "", brief(p), "[goal] 写" in buf.getvalue()],
   [True, ["写不进去的目标", 3, "active"], True])

# ============ F 路由：跑真 Handler，只是端口是随机的 ============
class FakeAsr(object):
    ready = False

    def info(self):
        return {"ready": False, "model": "假", "path_error": ""}


class FakeSpeak(object):
    def info(self):
        return {"ready": False, "voice": "假"}

    def speak(self, text):
        return {"ok": True}

    def stop(self):
        return {"ok": True, "was": "空闲"}


srv_box = []


def serve(s):
    car_chat.Handler.session, car_chat.Handler.asr = s, FakeAsr()
    car_chat.Handler.speaker = FakeSpeak()
    srv = ThreadingHTTPServer(("127.0.0.1", 0), car_chat.Handler)
    srv_box.append(srv)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return "http://127.0.0.1:%d" % srv.server_address[1]


def http_post(base, path, body, origin=None):
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json",
                                          "Connection": "close"})
    if origin:
        req.add_header("Origin", origin)
    try:
        with urllib.request.urlopen(req, timeout=8) as r:
            return r.status, json.loads(r.read().decode("utf-8", "replace"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8", "replace"))


def http_get(base, path):
    req = urllib.request.Request(base + path, headers={"Connection": "close"})
    with urllib.request.urlopen(req, timeout=8) as r:
        return json.loads(r.read().decode("utf-8", "replace"))


s = fresh(os.path.join(TMP, "goal_F.json"))
base = serve(s)
code, r = http_post(base, "/goal", {"text": "去找沙发"})
ck("F1 POST /goal 默认拿服务端轮数上限，回显生效那一份",
   [code, r["goal"]["turns_left"], r["goal"]["status"]],
   [200, car_chat.GOAL_MAX_TURNS, "active"])
code, r = http_post(base, "/goal", {"text": "  "})
ck("F2 空目标当场 400（不能把没写字存成目标）",
   [code, "text" in r.get("error", "")], [400, True])
ck("F3  连 text 字段都不发也是 400", http_post(base, "/goal", {})[0], 400)
code, r = http_post(base, "/goal", {"text": "长" * (car_chat.GOAL_MAX_CHARS + 1)})
ck("F4 超长的目标 400，且理由里带上限数字（它每轮都要重念，不是一份计划书）",
   [code, str(car_chat.GOAL_MAX_CHARS) in r.get("error", "")], [400, True])
for bad_v, why in (("5", "字符串"), (0, "0 轮"), (31, "超过上限"), (True, "布尔"), (-2, "负数")):
    code, r = http_post(base, "/goal", {"text": "去找沙发", "max_turns": bad_v})
    ck("F5 轮数写成 %s 不能当没写（%s）—— 静默忽略比拒绝危险" % (json.dumps(bad_v), why),
       [code, "1..%d" % car_chat.GOAL_MAX_TURNS in r.get("error", "")], [400, True])
code, r = http_post(base, "/goal", {"text": "去找沙发", "max_turns": 3.7})
ck("F6  3.7 折成 3 并且回显给的就是 3（回显对不上我以为发的那一份 = 事故）",
   [code, r["goal"]["turns_left"]], [200, 3])
st = http_get(base, "/state")
ck("F7 /state 的 chat.goal 挂着那一份", st["chat"]["goal"]["text"], "去找沙发")
ck("F8  暗号字样单独一格，且与代码里的常数同值（页面只读这里）",
   st["chat"]["goal_sentinel"], car_chat.GOAL_SENTINEL)
ck("F9 目标与两道逐轮闸各占各的格子（gates 里绝不掺 goal）",
   sorted(st["chat"]["gates"]), ["allow_tools", "max_angle", "max_distance"])
code, cap = http_post(base, "/chat", {"text": "报一下电量"})
ck("F10 /chat 的回显一个字段都没多（目标不改闸）",
   [code, sorted(cap)], [202, ["allow_tools", "max_angle", "max_distance", "ok"]])
ck("F11  发一句 /chat 之后，/state 里那一份的轮数掉了 1",
   http_get(base, "/state")["chat"]["goal"]["turns_left"], 2)
code, r = http_post(base, "/goal", {"clear": True})
ck("F12 清除回 200 并把原来那句话交还（人要知道丢了什么）",
   [code, r["goal"], r["cleared"]], [200, None, "去找沙发"])
ck("F13 没目标时再点清除不报错（幂等）", http_post(base, "/goal", {"clear": True})[0], 200)
ck("F14 跨站来源照样 403（这一页没鉴权，目标也是发指令的一张门）",
   http_post(base, "/goal", {"text": "别处来的"}, origin="http://evil.example")[0], 403)

# 忙的时候不许换目标：一轮跑到一半换，这一轮的账会算错头上
s = fresh(os.path.join(TMP, "goal_busy.json"))
base = serve(s)
SCRIPT["block"] = threading.Event()
s.start_turn("慢慢跑")
time.sleep(0.05)
ck("F15 正忙时 POST /goal 是 409（不是 400，也不是排队等它跑完）",
   http_post(base, "/goal", {"text": "换一个目标"})[0], 409)
ck("F16  清除随时可以：它是往安全方向走", http_post(base, "/goal", {"clear": True})[0], 200)
SCRIPT["block"].set()
SCRIPT["block"] = None
wait_idle(s)
ck("F17 跑完那一轮也没朝目标记账（目标半路被清掉，账不能记给一个不存在的东西）",
   goal_evs(s), [])

# ============ G 跨文件对账：页面发的字段、事件名、暗号字样 ============
CC_TXT = io.open(os.path.join(ROOT, "car_chat.py"), encoding="utf-8").read()
HTML = io.open(os.path.join(ROOT, "car_chat_ui.html"), encoding="utf-8").read()
branch = re.search(r'if path == "/goal":(.*?)if path == "/reset":', CC_TXT, re.S)
ck("G1 /goal 那段路由读得出来（读不到这份判据就是空的）", bool(branch), True)
srv_keys = set(re.findall(r'body\.get\("([a-z_]+)"', branch.group(1)))
gb = re.search(r'function goalBody\(\)\s*\{(.*?)\n\}', HTML, re.S)
ck("G2  页面的 goalBody 读得出来", bool(gb), True)
page_keys = (set(re.findall(r'\{(\w+)\s*:', gb.group(1))) |
             set(re.findall(r'\bb\.(\w+)\s*=', gb.group(1))))
ck("G3 页面发给 /goal 的字段服务端每个都读得到（服务端可以多，页面不许自己发明）",
   [bool(page_keys), page_keys <= srv_keys], [True, True])
ck("G4  清目标是同一个键，不是页面自创的第二套说法",
   ["clear" in srv_keys, "{clear: true}" in HTML], [True, True])
kinds = set(re.findall(r'self\.publish_goal\("([a-z_]+)"', CC_TXT))
ck("G5 服务端一共发这五种目标事件（改动加第六种时这里会响）",
   sorted(kinds), ["goal_cleared", "goal_done", "goal_expired", "goal_set", "goal_tick"])
cases = set(re.findall(r'case "(goal_\w+)"', HTML))
ck("G6 页面每一种目标事件都有渲染分支（漏一种就是发生了但人看不见）",
   [bool(kinds), kinds <= cases], [True, True])
ck("G7 页面没有自己抄一份暗号（字样只从 chat.goal_sentinel 拿）",
   [car_chat.GOAL_SENTINEL in HTML, "chat.goal_sentinel" in HTML], [False, True])
ck("G8  上限常数没抄进页面（30 只在服务端；页面填超了由服务端 400 那句说给人听）",
   ["GOAL_MAX" in HTML, 'placeholder="30"' in HTML], [False, False])
ck("G9 「按目标再跑一轮」发的是 /chat，不是一条新通道",
   [HTML.count('post("/chat"') >= 2, "runGoalTurn" in HTML], [True, True])
ck("G10 那一句是常量，且忙或没有 active 目标时点不动",
   ["const GOAL_NUDGE" in HTML, "busy || !on" in HTML], [True, True])
nudge = re.search(r'const GOAL_NUDGE = "([^"]+)"', HTML)
ck("G11 这句催促里不许出现工具名或距离（它是给人看的一句话，不是绕过闸的路）",
   any(w in nudge.group(1) for w in ("Move", "Tour", "米", "度")), False)

# ============ H 页面交出去的是数字还是字符串（迭代 13 那个 400 的教训） ============
node = shutil.which("node")


def js_func(name):
    m = re.search(r"^function %s\([^)]*\)\s*\{(.*?)^\}" % name, HTML, re.M | re.S)
    return m.group(0) if m else ""


if not node or not js_func("goalBody") or not js_func("goalTurns"):
    skips.append("没有 node 或抽不到页面那两个函数：这一节没证明交出去的是数字，只证明了字段名")
    print("SKIP 页面 JS 真跑那一节：%s" % skips[-1])
else:
    def run_js(vals):
        src = ("const V=%s, S=[];\nfunction $(id){return {value:(V[id]===undefined?'':V[id])};}\n"
               "function note(t){S.push(t);}\n%s\n%s\nS.push(JSON.stringify(goalBody()));\n"
               "console.log(JSON.stringify(S));" % (
                   json.dumps(vals, ensure_ascii=False), js_func("goalTurns"), js_func("goalBody")))
        p = subprocess.run([node, "-e", src], capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        if p.returncode != 0:
            raise AssertionError("页面那段 JS 在 node 里跑不通：%s" % p.stderr[:400])
        out = json.loads(p.stdout.strip().splitlines()[-1])
        return json.loads(out[-1]), out[:-1]      # 交回真对象：拒绝那一支是 null，不是"null"

    def types_of(d):
        return None if d is None else {k: type(v).__name__ for k, v in d.items()}

    ck("H1 正常那组：轮数交出去的是数字不是字符串（迭代 13 就是栽在类型上）",
       types_of(run_js({"goalText": "去找沙发", "goalTurns": "5"})[0]),
       {"text": "str", "max_turns": "int"})
    ck("H2  留空 = 这个字段整个不发（拿服务端上限），不是发个空字符串过去",
       "max_turns" in types_of(run_js({"goalText": "x", "goalTurns": ""})[0]), False)
    ck("H3  「abc」当场不发这一句", run_js({"goalText": "x", "goalTurns": "abc"})[0], None)
    ck("H4  0 轮当场不发（轮数没有 0 这种合法值）", run_js({"goalText": "x", "goalTurns": "0"})[0], None)
    ck("H5  半轮也当场不发：客户端只许比服务端严，不许更松",
       run_js({"goalText": "x", "goalTurns": "2.5"})[0], None)
    js, notes = run_js({"goalText": "  ", "goalTurns": "3"})
    ck("H6  空目标不发出去，而且当场说清为什么", [js, any("空的" in n for n in notes)], [None, True])
    ck("H7  99 轮页面照发（上限是服务端的常数）：由它 400，再由页面把那句念给人看",
       types_of(run_js({"goalText": "x", "goalTurns": "99"})[0]), {"text": "str", "max_turns": "int"})

for srv in srv_box:
    srv.shutdown()
shutil.rmtree(TMP, ignore_errors=True)
print("\n%s%s" % ("GOAL-TEST PASS" if not fails else "GOAL-TEST FAIL：%s" % fails,
                  ("\n（跳过：%s）" % "；".join(skips)) if skips else ""))
sys.exit(1 if fails else 0)
