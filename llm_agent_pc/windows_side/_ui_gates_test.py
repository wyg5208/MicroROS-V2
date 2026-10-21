# -*- coding: utf-8 -*-
"""页面上那排"本轮上闸"的离线判据：前端、聊天服务端、车侧工具表三方对得上。

为什么要单独一份：这个控件本身不碰车，最怕的是**三处各写各的**——
页面发了一个服务端不认的字段（会被**安静忽略**，于是人以为上了闸其实没有）、
预设里写了一个车上没有的工具名（白名单里错名字永远匹配不上，那一轮直接变成"什么都调不动"），
或者把"什么都不许调"和"没上闸"混成同一个值。这三种都是**看起来在工作、实际没生效**的那类故障，
拿真车验代价高（要故意让闸空转），拿纸面推理又验不到真代码，所以这里全用读盘解析跑。

第 9 节另有一层：把页面里那三个 JS 函数抽出来**交给 node 真跑一遍**。
2026-10-21 第一次浏览器实测就撞出"`input.value` 是字符串，服务端只认真数 → 当场 400"，
这件事用正则永远读不出来（字段名对、语义也对，只有类型错），只能让代码真算一次。

跑法：C:/Python314/python.exe -X utf8 C:/microros-agent/pc_side/_ui_gates_test.py（Windows 认正斜杠，省得跟转杠打架）
不连车、不连 :8000/:8020、不连大模型，全程零动作。
"""
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                 # C:\microros-agent
UI = os.path.join(ROOT, "car_chat_ui.html")
CHAT = os.path.join(ROOT, "car_chat.py")
BRIDGE = os.path.join(ROOT, "pc_side", "car_mcp_server.py")

fails = []
skips = []


def ck(label, got, want):
    ok = got == want
    print(("OK  " if ok else "FAIL ") + label)
    if not ok:
        print("      got=%r want=%r" % (got, want))
        fails.append(label)


def rd(p):
    return io.open(p, encoding="utf-8").read()


ui, chat, bridge = rd(UI), rd(CHAT), rd(BRIDGE)


def js_func(name):
    """取页面里那个函数的源码；页面用两段 CRLF/LF 都可能，所以按行首找、按 '\n}' 收尾。"""
    m = re.search(r"^function %s\([^)]*\)\s*\{(.*?)^\}" % name, ui, re.M | re.S)
    return m.group(0) if m else ""


# ---- 车侧真实存在的工具名：从 @mcp.tool 往下找第一个 def ----
def bridge_tools(text):
    out = []
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if "@mcp.tool" in line:
            for j in range(i + 1, min(i + 40, len(lines))):
                m = re.match(r"\s*def\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(", lines[j])
                if m:
                    out.append(m.group(1))
                    break
    return out


tools = bridge_tools(bridge)
ck("车侧工具表读得出 15 个名字（读不出来这份判据就是空的）", len(tools), 15)
ck("  里面确实有 Move / Tour / MapInfo", all(t in tools for t in ("Move", "Tour", "MapInfo")), True)

# ---- 1 页面发给 /chat 的字段，服务端必须每一个都认 ----
gb = re.search(r'function gateBody\(\)\s*\{(.*?)\n\}', ui, re.S)
assign = re.search(r'Object\.assign\(\{(\w+),\s*confirm', ui)
sent = {assign.group(1), "confirm"} if assign else set()
sent |= set(re.findall(r'\bb\.(\w+)\s*=', gb.group(1)) if gb else [])
server_keys = set(re.findall(r'body\.get\("(\w+)"\)', chat))
extra = sorted(sent - server_keys)
ck("页面发的字段服务端全都认（不认识的会被**安静忽略**——那是最坏的一种失败）", extra, [])
ck("  三个闸字段确实在页面上发出去", sorted(sent & {"max_distance", "max_angle", "allow_tools"}),
   ["allow_tools", "max_angle", "max_distance"])
ck("  服务端侧那三个也真的在收", {"max_distance", "max_angle", "allow_tools"} <= server_keys, True)

# ---- 2 预设里的工具名必须真的存在于车侧工具表 ----
opts = re.findall(r'<option value="([^"]*)"', ui)
preset_names = set()
for v in opts:
    if v and not v.startswith("__"):
        preset_names.update(x.strip() for x in v.split(","))
unknown = sorted(preset_names - set(tools))
ck("预设白名单里的每个工具名都真有其事（%s）" % "、".join(sorted(preset_names)), unknown, [])

# ---- 3 "只读"这个卖点不能骗人：预设里不许有动作工具 ----
risky = set(re.findall(r'^RISKY_TOOLS\s*=\s*\{([^}]*)\}', chat + rd(os.path.join(ROOT, "agent.py")),
                       re.M))
risky_names = set(re.findall(r'"(\w+)"', "".join(risky)))
ck("  agent.py 里 RISKY_TOOLS 读得出（读得出才敢拿它当判据）", len(risky_names) >= 4, True)
ck("只读预设里一个动作工具都没有", sorted(preset_names & risky_names), [])

# ---- 4 空 / [] / null 三种语义在 JS 里必须分得开 ----
gt = re.search(r'function gateTools\(\)\s*\{(.*?)\n\}', ui, re.S)
ck("gateTools() 找得到", bool(gt), True)
body_js = gt.group(1) if gt else ""
ck("  没挑预设 = null（= 这一句不限工具，字段整个不发）", 'if (v === "") return null;' in body_js, True)
ck("  挑了「什么都不许调」= 空数组，不是 null", 'if (v === "__none") return [];' in body_js, True)
ck("  留空的数字框**不发字段**（发 \"\" 过去服务端会 400，那是页面的错）",
   'if (raw === "") return [null, ""];' in js_func("gateNum")
   and "if (d[0] !== null) b.max_distance" in ui, True)

# ---- 5 状态栏要分开显示"本轮生效的闸"和"天花板"，混起来会让人误判下一句 ----
ck("服务端 /state 里 gates 与 caps 是两个键",
   [('"gates": s.gates' in chat), ('"caps": list(s.caps)' in chat)], [True, True])
ck("页面读这两个键（caps 没读就等于把上一轮的闸当成天花板）",
   [("chat.gates" in ui or "g.allow_tools" in ui) and ("chat.caps" in ui), True], [True, True])

# ---- 6 拦下时页面上必须有话，而且不许留一张像"动作已开始"的卡片 ----
ck('blocked 分支在（人看得见"没执行"）', 'case "blocked"' in ui, True)
ck("  而且那条文案指向的是能核对的地方，不是空口保证",
   "chat.gates" in re.search(r'case "blocked":(.*?); break', ui, re.S).group(1), True)

# ---- 7 页面上不许再刻会过期的数字（"0.5 m / 90°" 这类曾经错过一次） ----
stale = re.findall(r'\d+(?:\.\d+)?\s*(?:m|米)\s*/\s*\d+\s*(?:°|度)', ui)
ck("页面里没有硬编码的「上限 X 米 / Y 度」（现值只能从 /state 读）", stale, [])

# ---- 8 这一句的闸发完要能看见生效值，不然人只能信"我以为发了" ----
ck("202 回显里那三个值被念给人看", 'r.max_distance' in ui and 'r.allow_tools' in ui, True)
ck("  localStorage 存住选择（刷新后照旧，和确认开关一样）", "carChatGates" in ui, True)

# ---- 9 服务端只认真数，页面就必须交数字：这一段拿 node 真跑一遍 ----
# 浏览器实测（2026-10-21）撞出来的：`$("capDist").value` 是字符串 "0.4"，
# 一路 JSON.stringify 成 {"max_distance":"0.4"} 发给 :8020，服务端 _num() 只认
# int/float，于是**当场 400、整句没发出去**。字段名对、语义对、纸面全对，只有类型错。
num_rule = re.search(r"def _num\(v\):(.*?)\n\n", chat, re.S)
ck("服务端 _num() 读得出，并且它拒绝字符串（页面因此必须发数）",
   [bool(num_rule), "isinstance(v, (int, float))" in (num_rule.group(1) if num_rule else "")],
   [True, True])

node = shutil.which("node")
if not node:
    skips.append("第 9 节整节")
    print("SKIP 第 9 节（把页面 JS 交给 node 真跑）：本机没有 node —— 这轮**没有**证明页面交的是数字")
else:
    funcs = [js_func(n) for n in ("gateNum", "gateTools", "gateBody")]
    ck("  页面里 gateNum / gateTools / gateBody 三个函数都抽得出", all(funcs), True)
    harness = """
const V = %s;
const S = %s;
function $(id) { return {value: (id in V) ? V[id] : (S[id] || "")}; }
function note() {}
%s
const cases = [
  ["正常 0.4 / 20 + 只读", gateBodyOf("0.4", "20", "MapInfo,GetStatus", "")],
  ["两个框留空 = 不发这两个字段", gateBodyOf("", "", "MapInfo,GetStatus", "")],
  ["上限写 0（合法，是「劝」）", gateBodyOf("0", "0", "", "")],
  ["逗号写法 0,4 = 不当数", gateBodyOf("0,4", "", "", "")],
  ["字母 abc = 不当数", gateBodyOf("", "abc", "", "")],
  ["负数 -1 = 不当数", gateBodyOf("-1", "", "", "")],
  ["什么都没挑 = 空对象（不是 null）", gateBodyOf("", "", "", "")],
];
console.log(JSON.stringify(cases));
"""
    prelude = "\n".join(funcs) + """
function gateBodyOf(d, a, preset, custom) {
  V.capDist = d; V.capAng = a; V.capTools = custom;
  S.gatePreset = preset;
  return gateBody();
}
"""
    # gateTools() 读的是 $("gatePreset").value，上面的 $ 从 V/S 两处取，S 里放预设。
    harness = harness % ('{"capDist":"", "capAng":"", "capTools":""}', '{"gatePreset":""}', prelude)
    with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False, encoding="utf-8") as f:
        f.write(harness)
        path = f.name
    try:
        out = subprocess.run([node, path], capture_output=True, text=True, timeout=60)
    finally:
        os.unlink(path)
    if out.returncode != 0:
        ck("node 跑通页面那三个函数", out.stderr.strip()[:200], "")
    else:
        got = json.loads(out.stdout)
        labels = [c[0] for c in got]
        ck("  node 实跑的七种输入都跑到（顺序没乱）", labels,
           ["正常 0.4 / 20 + 只读", "两个框留空 = 不发这两个字段", "上限写 0（合法，是「劝」）",
            "逗号写法 0,4 = 不当数", "字母 abc = 不当数", "负数 -1 = 不当数", "什么都没挑 = 空对象（不是 null）"])
        vals = [c[1] for c in got]
        b1, b2, b3, b4, b5, b6, b7 = vals

        def types_of(b):
            return {k: type(v).__name__ for k, v in (b or {}).items()}

        ck("    正常那组：交出去的是**数字**不是字符串，白名单是列表",
           types_of(b1), {"max_distance": "float", "max_angle": "int", "allow_tools": "list"})
        ck("    上限数值没被改：0.4 与 20", [b1["max_distance"], b1["max_angle"]], [0.4, 20])
        ck("    留空那组：只有白名单，两个上限字段整个不出现",
           sorted(b2.keys()), ["allow_tools"])
        ck("    0 米 / 0 度发得出去（0 是合法值，不能被当成「没填」）",
           [b3.get("max_distance"), b3.get("max_angle"), "allow_tools" in b3], [0.0, 0.0, False])
        ck("    「0,4」当场不发（宁可不发，也不能发成没上闸的那一句）", b4, None)
        ck("    「abc」当场不发", b5, None)
        ck("    负数当场不发", b6, None)
        ck("    什么都没挑 = 空对象，句照发、按天花板跑（这跟「拦下不发」是两回事）", b7, {})

print("\n%s" % ("UI-GATES-TEST PASS" + ("（跳过 %s）" % "、".join(skips) if skips else "")
              if not fails else "UI-GATES-TEST FAIL：%s" % fails))
sys.exit(1 if fails else 0)
