#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MicroROS-V2 本地大模型智能体（纯标准库，无需 WSL/虚拟机/ROS）

链路与官方 OpenClaw 方案一致：本脚本 -> 车上 MCP 服务(streamable-http) -> ROS2 -> micro-ROS -> ESP32-S3

    python agent.py --self-test              # 离线自测，不碰车
    python agent.py --car 192.168.1.50 --list-tools
    python agent.py --car 192.168.1.50       # 交互模式
    python agent.py --car 192.168.1.50 --once "前进0.3米，然后左转90度"

环境变量可替代命令行参数：MICROROS_MCP_URL / LLM_BASE_URL / LLM_MODEL / LLM_API_KEY
"""

import argparse
import base64
import itertools
import json
import os
import re
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer

RISKY_TOOLS = {"Move", "TargetTrack", "FollowLine", "Navigation"}
ID_GEN = itertools.count(1)


class McpError(RuntimeError):
    pass


def _from_sse(body, msg_id):
    """在 text/event-stream 报文里找出本次请求的 JSON-RPC 响应。"""
    fallback = None
    for block in re.split(r"\n\s*\n", body):
        for line in block.splitlines():
            if not line.startswith("data:"):
                continue
            try:
                obj = json.loads(line[5:].strip())
            except ValueError:
                continue
            if not isinstance(obj, dict):
                continue
            if obj.get("id") == msg_id:
                return obj
            if "result" in obj or "error" in obj:
                fallback = fallback or obj
    return fallback


class McpClient:
    """MCP streamable-http 客户端：JSON-RPC over POST，兼容 json / SSE 两种响应。"""

    PROTOCOL_VERSION = "2025-03-26"

    def __init__(self, url, timeout=180.0):
        self.url = url
        self.timeout = timeout
        self.session_id = None
        self.protocol = None
        self.server_name = "?"

    def _rpc(self, method, params=None, msg_id=None):
        payload = {"jsonrpc": "2.0", "method": method}
        if params is not None:
            payload["params"] = params
        if msg_id is not None:
            payload["id"] = msg_id
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json, text/event-stream",
        }
        if self.session_id:
            headers["Mcp-Session-Id"] = self.session_id
        if self.protocol:
            headers["MCP-Protocol-Version"] = self.protocol
        req = urllib.request.Request(
            self.url, data=json.dumps(payload).encode("utf-8"), headers=headers, method="POST"
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                sid = resp.headers.get("Mcp-Session-Id")
                ctype = resp.headers.get("Content-Type") or ""
                raw = resp.read().decode("utf-8", "replace")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", "replace")[:300]
            raise McpError(f"MCP HTTP {exc.code} ({method}): {detail}") from exc
        except OSError as exc:
            raise McpError(f"无法连接 MCP 服务 {self.url}: {exc}") from exc
        if sid:
            self.session_id = sid
        if msg_id is None:  # notification，无响应体
            return None
        body = raw.strip()
        if not body:
            raise McpError(f"{method} 返回空响应")
        if "text/event-stream" in ctype or body.startswith(("event:", "data:")):
            obj = _from_sse(body, msg_id)
        else:
            obj = json.loads(body)
        if obj is None:
            raise McpError(f"{method} 的 SSE 响应里找不到 id={msg_id}")
        if "error" in obj:
            raise McpError(f"{method} 报错: {obj['error']}")
        return obj.get("result") or {}

    def connect(self):
        res = self._rpc(
            "initialize",
            {
                "protocolVersion": self.PROTOCOL_VERSION,
                "capabilities": {},
                "clientInfo": {"name": "microros-local-agent", "version": "1.0"},
            },
            msg_id=next(ID_GEN),
        )
        self.protocol = res.get("protocolVersion") or self.PROTOCOL_VERSION
        self.server_name = (res.get("serverInfo") or {}).get("name", "?")
        self._rpc("notifications/initialized")
        return res

    def list_tools(self):
        return self._fresh(lambda: self._list_tools())

    def call_tool(self, name, arguments):
        return self._fresh(lambda: self._call_tool(name, arguments))

    @staticmethod
    def _stale(exc):
        """服务端把我们报的会话号认出来了才叫"失效"；别的错误不要拿这个去重试。"""
        s = str(exc)
        return "unknown session" in s or "-32001" in s

    def _fresh(self, fn):
        # 重启 bridge 是日常操作（改一行 Python 就要重启），而会话号只活在那次握手裡。
        # 不自动重新握手的话，之后每一次工具调用都是 404，用户只会看到模型说"请稍后重试"。
        try:
            return fn()
        except McpError as exc:
            if not self._stale(exc):
                raise
            self.session_id = None
            self.protocol = None
            self.connect()
            return fn()

    def _list_tools(self):
        return self._rpc("tools/list", {}, msg_id=next(ID_GEN)).get("tools", [])

    def _call_tool(self, name, arguments):
        result = self._rpc(
            "tools/call", {"name": name, "arguments": arguments}, msg_id=next(ID_GEN)
        )
        texts = [
            p.get("text", "")
            for p in (result.get("content") or [])
            if p.get("type") == "text" and p.get("text")
        ]
        text = "\n".join(texts) or json.dumps(result, ensure_ascii=False)
        if result.get("isError"):
            text = "工具执行失败: " + text
        return text


def mcp_tools_to_openai(tools):
    """MCP 工具定义 -> OpenAI function calling 格式；返回 (工具列表, 名称映射)。"""
    out, name_map = [], {}
    for t in tools:
        raw_name = t["name"]
        safe = re.sub(r"[^A-Za-z0-9_-]", "_", raw_name) or "tool"
        name_map[safe] = raw_name
        schema = t.get("inputSchema") or {"type": "object", "properties": {}}
        out.append(
            {
                "type": "function",
                "function": {
                    "name": safe,
                    "description": (t.get("description") or raw_name).strip(),
                    "parameters": schema,
                },
            }
        )
    return out, name_map


def openai_chat(cfg, messages, tools=None, model=None):
    url = cfg.llm_base.rstrip("/") + "/chat/completions"
    payload = {
        "model": model or cfg.model,
        "messages": messages,
        "temperature": cfg.temperature,
        "stream": False,
    }
    if tools:
        payload["tools"] = tools
        payload["tool_choice"] = "auto"
    headers = {"Content-Type": "application/json"}
    if cfg.api_key:
        headers["Authorization"] = "Bearer " + cfg.api_key
    req = urllib.request.Request(
        url, data=json.dumps(payload).encode("utf-8"), headers=headers, method="POST"
    )
    try:
        with urllib.request.urlopen(req, timeout=cfg.llm_timeout) as resp:
            data = json.loads(resp.read().decode("utf-8", "replace"))
    except urllib.error.HTTPError as exc:
        raise RuntimeError(f"LLM HTTP {exc.code}: {exc.read().decode('utf-8','replace')[:300]}")
    except OSError as exc:
        raise RuntimeError(f"无法连接大模型服务 {url}: {exc}\n请先启动 LM Studio(Ollama) 并开启本地 Server") from exc
    if "choices" not in data:
        raise RuntimeError(f"大模型返回异常: {json.dumps(data, ensure_ascii=False)[:300]}")
    return data["choices"][0]["message"]


IMAGE_RE = re.compile(
    r"([A-Za-z]:[\\/][\w.\-\\/]+\.(?:png|jpe?g)"   # Windows 绝对路径
    r"|/[\w.\-\\/]+\.(?:png|jpe?g)"                 # POSIX 绝对路径
    r"|[\w.\-]+[/\\][\w.\-\\/]+\.(?:png|jpe?g))"    # 相对路径
)


def fetch_image(cfg, remote_path):
    """把车上图片取到本地临时目录；脚本若跑在车上则直接读原路径。"""
    if os.path.exists(remote_path):
        return remote_path
    if not cfg.ssh_target:
        return None
    tmp = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "cache", os.path.basename(remote_path)
    )
    os.makedirs(os.path.dirname(tmp), exist_ok=True)
    cmd = [
        "scp",
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout=8",
        f"{cfg.ssh_target}:{remote_path}",
        tmp,
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return tmp if proc.returncode == 0 and os.path.exists(tmp) else None


VISION_SYSTEM = (
    "你只是一个看图说话的描述器：给你一张小车相机拍的照片，你只描述画面内容"
    "（场景、主要物体、相机朝向偏高还是偏低、能判断时的物体大致距离）。"
    "你没有身体，不能移动相机或小车；不要说“我无法”，不要承诺、模拟或汇报任何动作，"
    "也不要向提问者反问。只输出对画面的客观描述。"
)


VISION_ASK = "描述这张照片里的场景、主要物体和大致距离。"


def describe_image(cfg, image_path):
    with open(image_path, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")
    mime = "image/png" if image_path.lower().endswith(".png") else "image/jpeg"
    msgs = [
        {"role": "system", "content": VISION_SYSTEM},
        {
            "role": "user",
            "content": [
                {"type": "text", "text": VISION_ASK},
                {"type": "image_url", "image_url": {"url": f"data:{mime};base64,{b64}"}},
            ],
        }
    ]
    return openai_chat(cfg, msgs, model=cfg.vision_model).get("content") or ""


def clamp_motion(cfg, name, args):
    """硬上限：小模型给出夸张参数时截断，避免车跑飞。"""
    if name != "Move":
        return args
    d = args.get("distance")
    a = args.get("angle")
    if isinstance(d, (int, float)):
        args["distance"] = max(-cfg.max_distance, min(cfg.max_distance, d))
    if isinstance(a, (int, float)):
        args["angle"] = max(-cfg.max_angle, min(cfg.max_angle, a))
    return args


def needs_confirm(cfg, name):
    return bool(cfg.confirm) and name in RISKY_TOOLS


class ConsoleHooks:
    """命令行用：事件打成原来那套 stdout，确认走 stdin。"""

    def event(self, ev):
        kind = ev["type"]
        if kind == "assistant":
            print("\n助手: " + ev["text"])
        elif kind == "call":
            print(f"  [{ev['step']}] 调用 {ev['tool']} "
                  f"{json.dumps(ev['args'], ensure_ascii=False)}")
        elif kind == "result":
            print("      结果: " + ev["text"].replace("\n", " ")[:200])
        elif kind == "badargs":
            print(f"  [{ev['step']}] {ev['tool']} -> {ev['text']}")
        elif kind == "limit":
            print("\n[已达到单轮最大工具调用次数，等待用户新指令]")

    def confirm(self, name, args):
        ans = input(f"  将执行 {name} {json.dumps(args, ensure_ascii=False)} ，回车 y 确认 / n 取消: ")
        return ans.strip().lower() in ("", "y", "yes")


SYSTEM_PROMPT = """你是运行在 Windows 笔记本上的 MicroROS-V2 智能小车控制助手，通过 MCP 工具指挥小车。

工作规则：
1. 需要动作或环境信息时必须调用工具，不要凭空描述结果。只有工具真实返回过的内容才算发生过；没有 Move 的工具结果就不能说车已经移动，没有 GetStatus 就不能报里程计坐标。
2. 想做视觉判断而上下文里没有画面时，先调用 SeeWhat 拍照；工具结果里若带"视觉模型描述"，据此回答。
3. Move 用里程计闭环，distance 单位米、angle 单位度；direction 只能是 forward/backward/turn_left/turn_right。
4. 雷达正前方约 44° 被云台自身遮挡。要用环境信息就先调 ScanRing 看环视；需要正前方净空调 CheckAhead（它原地左右各偏 30° 补测后回正）。没有测到的方向不等于空旷。
5. 单次动作不要超过 %s 米或 %s 度，复杂任务拆成多步，每步汇报结果。这两个上限就是机器真会执行的上限，别按更大的数规划。
6. 工具返回失败时先分清是哪一类，不要一律停手也不要一律重试：
   - 安全类（"收到停止指令"=人在按急停、"机器判定中止"=车自己发现位姿不可信、"雷达或里程计断了"、低压锁）：立刻停止后续动作并向用户说明原因。底盘速度受限、物理急停是关掉本服务的窗口。
   - 受阻类（文本里有"轮子在转但车没动"、"判定：被挡住"、"第N次脱困"）：车已经自己试过倒退或反向转，**不要把同一条命令原样再发**。按返回文本里的判定换一招，最多两招：转向最空旷方向、绕开这个方向、或调 Unstick（它会读四周雷达+地图、倒退并转向缺口）；还不通就 Stop 并告诉用户现场可能卡住的是什么（电源线、地毯边、轮子悬空、低矮障碍在雷达盲区）。
   重试任何工具都不要超过 2 次。
7. Move/MapSpin 的返回里带"命令 X 实测 Y"这类数字，它们是判断依据：实测远小于命令就是受阻，实测与命令相符但没走到位是速度问题，别混为一谈。
8. 全局状况（还剩哪儿没看、自己在地图哪里、雷达和地图对不对得上）先调 MapInfo 再决定去哪儿，它比 ScanRing 更适合当"下一步去哪儿"的依据。
9. 地点是有名字的：用户说"记住这里是XX""电视在哪""去厨房"时，Remember 把车此刻脚下钉成一个词（和这张地图存在一起，清空地图会一并忘掉），Forget 删掉一个。问某个地点在哪先调 MapInfo，它结尾按远近列出记住的地点，方位沿用 Move 的约定（正=向左），别自己换算。没记过的地点不要凭印象编，也不要为了回答就随手 Remember 一个。
10. 始终用简体中文回复，简洁明确。
11. 用户的话可能是语音转来的，同音字会被听错（网前=往前、后腿=后退、又转=右转、听车=停车、里米=厘米、依米=1米、建筑=建图）。程序已经纠过一批常见的，剩下的按最接近的行车意图理解，并在回复第一行写「按：往前走1米」这种格式说明你打算执行什么，让人一眼能否掉。方向词拿不准（比如"又/右"和"有"分不清）就不要动：先问一句，别猜着往墙上撞。
"""


def system_prompt(cfg):
    """把单次动作上限填进提示词，别留一份写死的副本。

    写死过一次：`--max-distance` 抬到 1.5 m 之后，模型仍按 0.5 m 把长动作拆成三步，
    白跑两倍趟数；反过来上限调小而提示词没跟着缩，它就会下一条根本执行不了的命令。
    """
    return SYSTEM_PROMPT % (cfg.max_distance, cfg.max_angle)


def run_turn(cfg, mcp, tools_spec, name_map, messages, user_text):
    """一轮用户指令：模型 <-> 工具，最多 max_steps 次调用。

    过程一律以结构化事件发给 cfg.hooks，命令行和网页共用同一条循环。
    """
    hooks = cfg.hooks
    messages.append({"role": "user", "content": user_text})
    hooks.event({"type": "user", "text": user_text})
    for step in range(cfg.max_steps):
        hooks.event({"type": "thinking", "step": step + 1})
        t_llm = time.monotonic()
        msg = openai_chat(cfg, messages, tools_spec)
        calls = msg.get("tool_calls") or []
        hooks.event({"type": "llm", "step": step + 1, "secs": time.monotonic() - t_llm})
        if not calls:
            content = (msg.get("content") or "").strip()
            if content:
                hooks.event({"type": "assistant", "text": content})
            messages.append({"role": "assistant", "content": content})
            return
        messages.append(
            {"role": "assistant", "content": msg.get("content") or "", "tool_calls": calls}
        )
        for call in calls:
            safe = call["function"]["name"]
            raw_args = call["function"].get("arguments") or "{}"
            real = name_map.get(safe, safe)
            try:
                args = json.loads(raw_args) if raw_args.strip() else {}
            except ValueError:
                result = f"参数不是合法 JSON: {raw_args}"
                hooks.event({"type": "badargs", "step": step + 1, "tool": real, "text": result})
                messages.append({"role": "tool", "tool_call_id": call["id"], "content": result})
                break
            args = clamp_motion(cfg, real, args)
            hooks.event({"type": "call", "step": step + 1, "tool": real, "args": args})
            secs = 0.0
            if cfg.dry_run:
                result = "dry-run：未真正下发给机器人"
            elif needs_confirm(cfg, real) and not hooks.confirm(real, args):
                result = ("用户拒绝了这个动作。这是人的决定，不是故障：不要再重复请求同一个动作，"
                          "也不要换个说法再来一次。请直接问用户想怎么改。")
                hooks.event({"type": "cancelled", "step": step + 1, "tool": real, "args": args})
            else:
                t0 = time.monotonic()
                try:
                    result = mcp.call_tool(real, args)
                except McpError as exc:
                    result = f"MCP 调用异常: {exc}"
                secs = time.monotonic() - t0
            if cfg.vision_model and real in ("SeeWhat", "AdjustCameraView"):
                hit = IMAGE_RE.search(result)
                local = fetch_image(cfg, hit.group(1)) if hit else None
                if local:
                    try:
                        result += "\n视觉模型描述: " + describe_image(cfg, local)
                        hooks.event({"type": "image", "tool": real, "path": local})
                    except RuntimeError as exc:
                        result += f"\n[视觉模型失败: {exc}]"
                else:
                    result += "\n[图像未能取到本地]"
            hooks.event({"type": "result", "step": step + 1, "tool": real,
                         "args": args, "text": result, "secs": secs})
            messages.append({"role": "tool", "tool_call_id": call["id"], "content": result})
    else:
        hooks.event({"type": "limit"})


def build_argparser():
    p = argparse.ArgumentParser(description="MicroROS-V2 本地大模型智能体（纯标准库）")
    p.add_argument("--car", help="车机 IP，例如 192.168.1.50")
    p.add_argument("--mcp-url", default=os.environ.get("MICROROS_MCP_URL"),
                   help="MCP 端点，默认 http://<car>:8000/mcp")
    p.add_argument("--llm-base", default=os.environ.get("LLM_BASE_URL", "http://127.0.0.1:1234/v1"),
                   help="OpenAI 兼容端点：LM Studio :1234/v1，Ollama :11434/v1")
    p.add_argument("--model", default=os.environ.get("LLM_MODEL", "qwen3-14b"), help="工具调用模型")
    p.add_argument("--vision-model", default=os.environ.get("VISION_MODEL") or None,
                   help="本地视觉模型名，设置后 SeeWhat 图像会自动描述给大脑")
    p.add_argument("--api-key", default=os.environ.get("LLM_API_KEY", "local"),
                   help="本地服务填任意非空值即可")
    p.add_argument("--ssh-target", default=os.environ.get("CAR_SSH"),
                   help="车机 ssh 目标，用于 scp 取图，例如 jetson@192.168.1.50")
    p.add_argument("--once", help="执行单条指令后退出")
    p.add_argument("--list-tools", action="store_true", help="只列出车上 MCP 工具")
    p.add_argument("--self-test", action="store_true", help="离线自测：内置 mock MCP + mock LLM")
    p.add_argument("--dry-run", action="store_true", help="只打印工具调用，不下发给车")
    p.add_argument("--no-confirm", dest="confirm", action="store_false", help="关闭移动动作人工确认")
    p.add_argument("--max-steps", type=int, default=10)
    p.add_argument("--max-distance", type=float, default=1.0, help="单次 Move 最大位移(米)")
    p.add_argument("--max-angle", type=float, default=360.0, help="单次 Move 最大转角(度)")
    p.add_argument("--temperature", type=float, default=0.3)
    p.add_argument("--llm-timeout", type=float, default=300.0)
    p.add_argument("--mcp-timeout", type=float, default=180.0)
    p.set_defaults(hooks=ConsoleHooks())
    return p


class MockHandler(BaseHTTPRequestHandler):
    """自测用：同时冒充车上 MCP 服务和本地大模型。"""

    def log_message(self, *args):
        pass

    def _send(self, code, body, extra):
        data = body.encode("utf-8")
        self.send_response(code)
        for k, v in extra.items():
            self.send_header(k, v)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        req = json.loads(self.rfile.read(length).decode("utf-8"))
        method, mid = req.get("method"), req.get("id")
        if self.path.endswith("/mcp"):
            if method == "initialize":
                result = {"protocolVersion": "2025-03-26", "capabilities": {"tools": {}},
                          "serverInfo": {"name": "ROS-MCP-Service(mock)", "version": "0"}}
                self._send(200, json.dumps({"jsonrpc": "2.0", "id": mid, "result": result}),
                           {"Content-Type": "application/json", "Mcp-Session-Id": "mock-session"})
            elif method == "notifications/initialized":
                self._send(202, "", {})
            elif method == "tools/list":
                tools = [
                    {"name": "Move",
                     "description": "Move the robot chassis by a specified distance or rotate by a specified angle using odometry feedback. Supported directions: forward, backward, turn_left, turn_right.",
                     "inputSchema": {"type": "object", "properties": {
                         "direction": {"type": "string", "enum": ["forward", "backward", "turn_left", "turn_right"]},
                         "distance": {"type": "number"}, "angle": {"type": "number"}},
                         "required": ["direction"]}},
                    {"name": "SeeWhat",
                     "description": "Capture an image from the camera and return the image save path",
                     "inputSchema": {"type": "object", "properties": {}}},
                    {"name": "GetMapMapping",
                     "description": "Get the mapping relationship between all place names and their corresponding symbols",
                     "inputSchema": {"type": "object", "properties": {}}},
                ]
                # 故意用 SSE 返回，验证事件流解析
                body = "event: message\r\ndata: " + json.dumps(
                    {"jsonrpc": "2.0", "id": mid, "result": {"tools": tools}}) + "\r\n\r\n"
                self._send(200, body, {"Content-Type": "text/event-stream"})
            elif method == "tools/call":
                tool = req["params"]["name"]
                payload = req["params"]["arguments"]
                if tool == "SeeWhat":
                    body_text = "image saved to /home/yahboom/microros_ws/cache/image.png"
                elif tool == "GetMapMapping":
                    body_text = "{'客厅': 'A', '书桌': 'B'}"
                else:
                    body_text = json.dumps({"mock": "moved", "args": payload})
                result = {"content": [{"type": "text", "text": json.dumps(
                    {"execution_result": "success", "reason": body_text})}], "isError": False}
                self._send(200, json.dumps({"jsonrpc": "2.0", "id": mid, "result": result}),
                           {"Content-Type": "application/json"})
            else:
                self._send(404, "", {})
        elif self.path.endswith("/chat/completions"):
            got_tool = any(m.get("role") == "tool" for m in req["messages"])
            if got_tool:
                message = {"role": "assistant", "content": "机器人已前进 0.3 米（模拟执行成功）。"}
            else:
                message = {"role": "assistant", "content": "我先让机器人前进。",
                           "tool_calls": [{"id": "call_1", "type": "function", "function": {
                               "name": "Move",
                               "arguments": json.dumps({"direction": "forward", "distance": 0.3,
                                                       "angle": 0.0})}}]}
            self._send(200, json.dumps({"choices": [{"message": message}]}),
                       {"Content-Type": "application/json"})
        else:
            self._send(404, "", {})


def self_test(cfg):
    server = HTTPServer(("127.0.0.1", 0), MockHandler)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    cfg.mcp_url = f"http://127.0.0.1:{port}/mcp"
    cfg.llm_base = f"http://127.0.0.1:{port}/v1"
    cfg.confirm = False
    try:
        mcp = McpClient(cfg.mcp_url, timeout=15.0)
        info = mcp.connect()
        tools = mcp.list_tools()
        tools_spec, name_map = mcp_tools_to_openai(tools)
        messages = [{"role": "system", "content": system_prompt(cfg)}]
        run_turn(cfg, mcp, tools_spec, name_map, messages, "前进 0.3 米")
        transcript = json.dumps(messages, ensure_ascii=False)
        assert info["serverInfo"]["name"].endswith("(mock)"), "initialize 未拿到 serverInfo"
        assert tools and tools_spec[0]["function"]["name"] == "Move", "tools/list 解析失败"
        assert "moved" in transcript, "工具调用结果未回传给模型"
        assert "0.3 米" in transcript, "最终回复缺失"
        for text, want in [
            ("已拍摄，图像路径: C:\\microros-agent\\pc_side\\cache\\see_1.jpg (4806 bytes)",
             r"C:\microros-agent\pc_side\cache\see_1.jpg"),
            ("image saved to /home/yahboom/microros_ws/cache/image.png",
             "/home/yahboom/microros_ws/cache/image.png"),
        ]:
            hit = IMAGE_RE.search(text)
            assert hit and hit.group(1) == want, "图像路径解析失败: %r -> %r" % (text, hit)
        print("\n[self-test] PASS：MCP 握手/SSE 解析/工具调用回环/图像路径(含 Windows) 均正常")
        return 0
    except Exception as exc:
        print(f"\n[self-test] FAIL: {type(exc).__name__}: {exc}")
        return 1
    finally:
        server.shutdown()


def main():
    cfg = build_argparser().parse_args()
    if cfg.self_test:
        sys.exit(self_test(cfg))
    if not cfg.mcp_url:
        if not cfg.car:
            print("缺少目标车 IP：用 --car 192.168.1.50 或 --mcp-url；也可先跑 --self-test")
            sys.exit(2)
        cfg.mcp_url = f"http://{cfg.car}:8000/mcp"
    mcp = McpClient(cfg.mcp_url, timeout=cfg.mcp_timeout)
    print(f"连接 MCP 服务: {cfg.mcp_url}")
    try:
        mcp.connect()
        tools = mcp.list_tools()
    except McpError as exc:
        print(f"[连接失败] {exc}\n"
              "检查：① 先启动「启动小车服务.bat」（本机 :8000 的 MCP 服务）"
              "② micro-ROS Agent 是否在 8090/udp 运行 ③ 车上电并连同一个 WiFi")
        sys.exit(1)
    tools_spec, name_map = mcp_tools_to_openai(tools)
    print(f"MCP 服务: {mcp.server_name}  会话: {mcp.session_id}  工具 {len(tools)} 个:")
    for t in tools:
        print(f"  - {t['name']}: {(t.get('description') or '').strip().splitlines()[0][:80]}")
    if cfg.list_tools:
        return
    if not tools:
        print("车上没有注册任何工具，请确认已运行 openclaw_service.launch.py")
        sys.exit(1)

    print(f"\n大模型: {cfg.model} @ {cfg.llm_base}"
          + (f"   视觉模型: {cfg.vision_model}" if cfg.vision_model else "   (视觉模型未启用)"))
    if cfg.dry_run:
        print("[dry-run] 工具调用只打印，不会下发给车")
    messages = [{"role": "system", "content": system_prompt(cfg)}]
    if cfg.once:
        run_turn(cfg, mcp, tools_spec, name_map, messages, cfg.once)
        return
    print("输入指令（/quit 退出，/reset 清空上下文，/dry 切换试运行）\n")
    while True:
        try:
            text = input("你: ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not text:
            continue
        if text.lower() in ("/quit", "/exit", "/q"):
            break
        if text == "/reset":
            messages[:] = [messages[0]]
            print("[上下文已清空]")
            continue
        if text == "/dry":
            cfg.dry_run = not cfg.dry_run
            print(f"[dry-run = {cfg.dry_run}]")
            continue
        try:
            run_turn(cfg, mcp, tools_spec, name_map, messages, text)
        except (RuntimeError, McpError) as exc:
            print(f"[出错] {exc}")


if __name__ == "__main__":
    main()
