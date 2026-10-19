#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MicroROS-V2 网页聊天前端（纯标准库）：把 agent.py 那条循环搬到浏览器里。

    python car_chat.py --mcp-url http://127.0.0.1:8000/mcp \
        --llm-base http://127.0.0.1:11434/v1 --model qwen3.5:9b --vision-model qwen3.5:9b \
        --max-distance 1.5 --max-angle 90

单次位移/转角的上限就在这里传（启动脚本与 _relaunch_chat.ps1 各写了一份，改要一起改）；
提示词第 5 条的那句"不要超过 X 米"是 agent.system_prompt(cfg) 按它现填的，不会跟实际值说谎。

这一层**不开任何 ROS 订阅**：下发动作走车上 MCP(:8000)，看轮子到底动没动读网页控制台
(:8010)，所以它不需要 _rosrun.bat，也不需要为了它重启 bridge。
"""

import itertools
import json
import os
import queue
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import agent  # noqa: E402

UI_FILE = os.path.join(HERE, "car_chat_ui.html")
CACHE_DIR = os.path.realpath(os.path.join(HERE, "pc_side", "cache"))
CONSOLE = "http://127.0.0.1:8010"
RING_MAX = 240          # 保留最近多少条事件，供刷新/新标签页补齐
SSE_REPLAY = 80
STATE_TTL = 0.30        # /api/state 的极短缓存：页面每秒问一次，别把控制台问烦
MOTION_TOOLS = ("Move", "Tour", "MapSpin")

# 语音这条路上没有任何转码环节：浏览器录 16 kHz 单声道、自己拼 WAV 头，
# whisper.cpp 吃 WAV，SAPI 用 PowerShell 出声。本机没有 ffmpeg，装了也白装。
ASR_EXE = os.path.join(HERE, "asr", "bin", "Release", "whisper-cli.exe")
ASR_MODEL = os.path.join(HERE, "asr", "models", "ggml-base.bin")
ASR_TMP = os.path.join(HERE, "asr", "req")
ASR_MAX_BYTES = 12 * 1024 * 1024      # 16k/16bit 单声道约 6 分钟，点着说话的片段远到不了
ASR_NOISE = ("whisper_", "load_model", "system_info", "ggml", "main:", "|>", "[00:")
# whisper 听中文很爱吐繁体：40 句真指令里 26 句带繁体（前進/右轉/電池）。
# 塞一句简体中文进 initial prompt 能把它压到 5/40（实测见 _voice_test.py 的说明），剩下这几个
# 手抄的字对表负责抹平。刻意只做"这个场合真的会出现的字"，不是完整繁简转换 —— 表里没有的字
# 原样留着，比猜错一个好。
ASR_T2S_PAIRS = (
   "點点電电頭头圖图開开關关間间還还邊边說说對对時时實实動动車车馬马鳥鸟魚鱼龍龙風风聲声聽听見见觀观覺觉記记設设語语詞词試试話话讀读寫写機机腦脑憶忆遠远選选遺遗運运過过達达術术樹树"
   "葉叶藥药飯饭館馆駐驻騎骑鮮鲜黃黄黨党齊齐齡龄東东許许論论證证訴诉認认識识變变讓让誰谁調调談谈議议護护報报場场壞坏塊块堅坚審审屬属賣卖寧宁寬宽導导盡尽據据數数檢检沒没測测牆墙現现"
   "環环畫画發发礙碍築筑糧粮納纳線线繞绕裡里裏里輕轻轉转進进閉闭來来兩两剛刚麼么樣样後后們们個个將将專专業业極极構构優优備备內内冊册養养緊紧紀纪純纯細细結结給给統统繼继績绩總总練练"
   "縣县連连遊游適适週周遲迟鄉乡醫医釋释錢钱錯错錄录長长門门問问閥阀雲云靜静項项順顺須须預预頻频類类飛飞驗验驚惊體体鬥斗麗丽麥麦鳴鸣鴉鸦髮发張张嗎吗攝摄戰战補补傾倾網网頁页彎弯圓圆"
   "頂顶樓楼層层園园輪轮離离訊讯應应擊击響响顯显軟软視视衛卫裝装隨随從从會会經经歷历嚴严勢势質质願愿顧顾驅驱駛驶軌轨態态準准確确啟启傳传聯联樂乐務务條条餘余處处係系寶宝"
)
_T2S = {}
for _i in range(0, len(ASR_T2S_PAIRS) - len(ASR_T2S_PAIRS) % 2, 2):
    _pair = ASR_T2S_PAIRS[_i:_i + 2]
    assert _pair[0] != _pair[1], "字对表里有一对不像一对：%r" % _pair
    assert _pair[0] not in _T2S, "字对表里有重复的繁体字：%s" % _pair[0]
    _T2S[_pair[0]] = _pair[1]
assert len(ASR_T2S_PAIRS) % 2 == 0, "字对表长度不是偶数，抄漏了一个字"
assert not (set(_T2S) & set(_T2S.values())), "字对表里有方向可疑的字（繁体同时当了简体）"
# 一句领域内的简体中文当 initial prompt：它同时纠正繁体和"里米/建築"这类同音错字
# （实测内容不符从 33/40 降到 24/40）。--asr-prompt "" 可以关掉，代价是繁体回到 26/40。
ASR_PROMPT = ("向前进五十厘米，原地左转三十度，开始建图，停车，绕房间走一圈看看哪里还没探测，"
              "后退二十厘米，扫描环境，打开摄像头，把地图保存到文件，电量还剩多少，"
              "检查雷达数据，清空上下文，回到起点。")
SPEAK_PS1 = os.path.join(HERE, "_speak.ps1")
SPEAK_TMP = os.path.join(HERE, "asr", "say")

def to_simplified(text):
    """只换表里的字，其余原样 —— 把一个不该换的字换错，比留一个繁体字严重。"""
    return "".join(_T2S.get(c, c) for c in text)


# 同音纠正在给模型之前做一遍。base 档听中文短句就是会这样错（下面每一条都站在实测证据上：
# 40 句语料的 rows.json 里真出现过 少描/建筑/射向头/善停/轻空/粮米/驾驻/纳面/延强边/里米，
# "网前走依米""又转""听车"是用户实测语音那一轮报回来的）。为什么不干脆让大模型去猜：
# 这条路要为几个同音字多花一次推理不值，而且纠对了要能给人看见，猜对了只是一句"好的"。
# 规则刻意**绑短语**不绑单字：这个场合根本不会出现"建筑/听车/依米"这种词组，替换几乎不可能误伤；
# 换成单字表就会把"作/赚/里/听"到处乱改 —— 那是真把话讲坏。
# 剩下确实纠不了的（"继续"听成"进去"）交给提示词第 10 条，让模型说出它按什么执行。
ASR_FIX = (
    ("延强边", "沿墙边"), ("言墙边", "沿墙边"), ("强边", "墙边"),
    ("少描", "扫描"), ("搜妙", "扫描"), ("移下", "一下"),
    ("射向头", "摄像头"), ("观比", "关闭"),
    ("建筑", "建图"), ("建土", "建图"),
    ("善停", "暂停"), ("轻空", "清空"), ("检察", "检查"),
    ("驾驻", "卡住"), ("假住", "卡住"), ("你的未知", "你的位置"), ("纳面", "那面"),
    ("专议", "转一"), ("补药痘", "不要动"), ("地图等", "地图的"),
    ("粮米", "两米"), ("里米", "厘米"), ("李米", "厘米"), ("黎米", "厘米"),
    ("网前", "往前"), ("王前", "往前"), ("望前", "往前"), ("枉前", "往前"),
    ("依米", "1米"), ("以米", "1米"), ("已米", "1米"), ("伊米", "1米"),
    ("后腿", "后退"), ("候退", "后退"), ("厚退", "后退"),
    ("又转", "右转"), ("右赚", "右转"), ("右撰", "右转"), ("又边", "右边"), ("又面", "右面"),
    ("作转", "左转"), ("佐转", "左转"), ("左赚", "左转"), ("左撰", "左转"), ("作边", "左边"),
    ("听车", "停车"), ("听下", "停下"), ("听止", "停止"), ("厅车", "停车"),
    ("赚圈", "转圈"), ("撰圈", "转圈"), ("原第", "原地"), ("砍看", "看看"),
    ("排照", "拍照"), ("派照", "拍照"), ("拍赵", "拍照"),
)
# 刻意**不进表**的两类（都实测到过）：
#   "继续"听成"倾斜" —— 倾斜是本车说得出的词（"车是不是倾斜了"），换掉就是改坏人的话；
#   "暂停"听成"《战体》" —— 带书名号的整段是 whisper 幻觉，不是同音字，表里没有任何正确写法可比。
# 这两种留给提示词第 10 条：让模型说出它按什么执行，人一眼能否掉。



def fix_command(text):
    """返回 (纠正后的文本, 改了哪几处)。没命中就原样返回，绝不留痕。

    要过到**不再变化**为止（最多 4 遍）：实测"沿墙边"会被听成"言强边"，
    一条走完才出现另一条能匹配的字 —— 只走一遍就停在"言墙边"，等于纠了一半。
    """
    hits = []
    for _ in range(4):
        changed = False
        for bad, good in ASR_FIX:
            if bad != good and bad in text:
                text = text.replace(bad, good)
                hits.append("%s→%s" % (bad, good))
                changed = True
        if not changed:
            break
    return text, hits


def looks_like_speech(text):
    """转写结果里到底有没有"话"。

    刻意不用 `isalnum()`：注音符号 ㄅㄇ 也是 alnum，而 whisper 听不清一句短指令时恰恰会吐一串
    注音（实测"停车"→"ㄇㄇㄇ…"）。这里只认汉字方块、数字和拉丁字母。
    """
    return any("\u4e00" <= c <= "\u9fff" or c.isdigit() or ("a" <= c.lower() <= "z")
               for c in text)


class Busy(RuntimeError):
    pass


def non_ascii_path(*paths):
    """挑出路径里带中文（任何非 ASCII）的那几条。

    whisper.cpp 的 Windows 版按当前 ANSI 码页读 argv 再去开文件，路径里有中文时
    模型直接打不开：实测 Git Bash 里退 127，Python 里退 0xC0000409（fail-fast），
    两种都跟"没听清"毫无关系，只会让人以为是模型坏了。所以把它当成安装位置错误，
    在状态里说清楚，而不是等子进程崩。
    """
    return [p for p in paths if any(ord(c) > 127 for c in p)]


class Asr:
    """whisper.cpp 的一次性子进程。

    刻意不起常驻服务、不加依赖：代价是每次都要重载入模型，实测 base 在 i9-14900HX
    上一条 2-4 秒的中文指令 0.5-1.2 秒（含载入），small 1.7-3.6 秒，
    medium-q5_0 要 7-8 秒却没有更准 —— 所以默认 base。
    一次只跑一条：转写很吃 CPU，排队比并发诚实。
    """

    def __init__(self, exe=ASR_EXE, model=ASR_MODEL, lang="zh", threads=12, timeout=25.0,
                 prompt=ASR_PROMPT):
        self.exe, self.model, self.lang = exe, model, lang
        self.threads, self.timeout = int(threads), float(timeout)
        self.prompt = prompt or ""
        self.lock = threading.Lock()
        self.busy = False
        self.last = None            # {"text","secs"} 给页面/日志看一眼
        os.makedirs(ASR_TMP, exist_ok=True)

    @property
    def ready(self):
        return os.path.isfile(self.exe) and os.path.isfile(self.model)

    def path_error(self):
        bad = non_ascii_path(self.exe, self.model, ASR_TMP)
        if not bad:
            return None
        return ("whisper 装在了带中文的路径下，它读不了这种路径（会直接崩，不是识别错）："
                + "、".join(bad) + "。把整个 asr 目录挪到纯英文路径（例如 C:\\microros-agent\\asr）。")

    def info(self):
        return {"ready": self.ready, "busy": self.busy, "lang": self.lang,
                "prompt": bool(self.prompt), "exe": self.exe,
                "model": os.path.basename(self.model), "path_error": self.path_error()}

    def transcribe(self, data):
        if not self.ready:
            return {"ok": False, "error": "没有 whisper-cli 或模型：%s / %s（先跑 asr\\_asr_setup.py）"
                    % (self.exe, self.model)}
        err = self.path_error()
        if err:
            return {"ok": False, "error": err}
        if len(data) < 44:
            return {"ok": False, "error": "音频太短（%d 字节），按住再说久一点" % len(data)}
        if len(data) > ASR_MAX_BYTES:
            return {"ok": False, "error": "音频太大（%d 字节），最长约 6 分钟" % len(data)}
        # 只认 WAV：RIFF 头不对就说明前端拼错了，别把它硬塞给 whisper 再回一句"没听清"
        if data[:4] != b"RIFF" or data[8:12] != b"WAVE":
            return {"ok": False, "error": "不是 WAV（RIFF/WAVE），浏览器那段录音代码有问题"}
        if not self.lock.acquire(blocking=False):
            return {"ok": False, "error": "上一条还在转写，等一下再说"}
        path = os.path.join(ASR_TMP, "req-%d.wav" % int(time.time() * 1000))
        t0 = time.perf_counter()
        try:
            with open(path, "wb") as fh:
                fh.write(data)
            argv = [self.exe, "-m", self.model, "-f", path, "-l", self.lang,
                    "-t", str(self.threads), "-nt", "-sns"]
            if self.prompt:
                argv += ["--prompt", self.prompt, "--carry-initial-prompt"]
            r = subprocess.run(argv, capture_output=True, timeout=self.timeout)
            secs = time.perf_counter() - t0
            lines = []
            for ln in (r.stdout or b"").decode("utf-8", "replace").splitlines():
                s = ln.strip()
                if s and not s.startswith(ASR_NOISE):
                    lines.append(s)
            text = to_simplified(" ".join(lines).strip())
            # 一句短指令最容易被 whisper 编成注音符号或纯标点（实测"停车"→"ㄇㄇㄇ…"，
            # 加 -sns 之后没了）；这种句子填进输入框只会浪费人一轮，直接说没听清。
            if not looks_like_speech(text):
                text = ""
            if not text:
                self.last = None
                return {"ok": False, "error": "没听清（转写为空）", "secs": round(secs, 2),
                        "raw": (r.stderr or b"").decode("utf-8", "replace")[-200:]}
            heard = text
            text, hits = fix_command(heard)
            self.last = {"text": text, "secs": round(secs, 2),
                         "heard": heard if hits else None}
            out = {"ok": True, "text": text, "secs": round(secs, 2)}
            if hits:
                out.update({"heard": heard, "fixes": hits})
            return out
        except subprocess.TimeoutExpired:
            return {"ok": False, "error": "转写超时（%.0f s）" % self.timeout}
        except OSError as exc:
            return {"ok": False, "error": "起不了 whisper-cli：%s" % exc}
        finally:
            self.busy = False
            self.lock.release()
            try:
                os.remove(path)
            except OSError:
                pass


class Speaker:
    """SAPI 朗读：起一个 powershell 念完就退，急停时掐得掉。

    文本经临时 UTF-8 文件传给 .ps1，不塞进命令行 —— PowerShell 命令行里带中文
    会被上游 shell 的编码吃掉（这条在第 6 部分的聊天探测里踩过）。
    """

    def __init__(self, ps1=SPEAK_PS1, voice="", max_chars=400):
        self.ps1, self.voice, self.max_chars = ps1, voice, int(max_chars)
        self.tmpdir = SPEAK_TMP
        self.lock = threading.Lock()
        self.proc = None
        self.last = ""
        self.on = True
        os.makedirs(self.tmpdir, exist_ok=True)

    @property
    def ready(self):
        return self.on and os.path.isfile(self.ps1)

    def info(self):
        return {"ready": self.ready, "speaking": self.proc is not None,
                "voice": self.voice or "默认嗓", "last": self.last, "ps1": self.ps1}

    @staticmethod
    def clip(text, max_chars):
        """空白折成一句、截到 max_chars —— 一句几千字的报告不该从头念到尾。"""
        return " ".join(str(text or "").split())[:int(max_chars)]

    def speak(self, text):
        text = self.clip(text, self.max_chars)
        if not text:
            return {"ok": False, "error": "没有要念的字"}
        if not self.ready:
            return {"ok": False, "error": "朗读没开或找不到 %s" % self.ps1}
        self.stop()
        path = os.path.join(self.tmpdir, "say-%d.txt" % int(time.time() * 1000))
        try:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(text)
            argv = ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
                    "-File", self.ps1, "-Path", path]
            if self.voice:
                argv += ["-Voice", self.voice]
            with self.lock:
                t0 = time.time()
                self.proc = subprocess.Popen(argv, stdout=subprocess.DEVNULL,
                                             stderr=subprocess.PIPE)
                proc = self.proc
        except OSError as exc:
            return {"ok": False, "error": "起不了朗读进程：%s" % exc}

        def reap():
            err = proc.stderr.read().decode("utf-8", "replace") if proc.stderr else ""
            code = proc.wait()
            secs = time.time() - t0
            with self.lock:
                if self.proc is proc:
                    self.proc = None
            try:
                os.remove(path)
            except OSError:
                pass
            if getattr(proc, "_asked", False):
                return
            tail = ("：" + err.strip()[-160:]) if err.strip() else ""
            self.last = "念了 %.1f s（%d 字，rc=%s%s）" % (secs, len(text), code, tail)
            # 几十个字正常要好几秒；零点几秒就退＝八成没接到出声设备。
            # 以前只放过“拒绝”字样，结果念不出声时日志一片安静，把安静当成了成功。
            if code != 0 or secs < 0.15 * len(text):
                self.last = "这句可能没出声：" + self.last
                sys.stderr.write("[speak] %s\n" % self.last)

        threading.Thread(target=reap, daemon=True).start()
        return {"ok": True, "chars": len(text)}

    def stop(self):
        """掐掉当前这句，并顺手清掉排队念的 —— 车停了话也不该继续说。"""
        with self.lock:
            proc, self.proc = self.proc, None
        if proc is None:
            return {"ok": True, "was": None}
        proc._asked = True
        try:
            proc.terminate()
            try:
                proc.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                proc.kill()
        except OSError:
            pass
        return {"ok": True, "was": "stopped"}


class Session:
    """一条对话 + 一次只允许一个动作回合 + 一把要人点头的确认闸。"""

    def __init__(self, cfg, mcp, tools_spec, name_map, console=CONSOLE, approve_timeout=60.0):
        self.cfg, self.mcp = cfg, mcp
        self.tools_spec, self.name_map = tools_spec, name_map
        self.console, self.approve_timeout = console, approve_timeout
        self.messages = [{"role": "system", "content": agent.system_prompt(cfg)}]
        self.busy = False
        self.turns = 0
        self.started = time.time()
        self.ring = []
        self.subs = []
        self.lock = threading.Lock()          # 保护 ring / subs / seq
        self.seq = itertools.count(1)
        self.pending = {}                     # 待确认：id -> {"ev","allow","tool","args"}
        self.refused = False                  # 本轮是否已经被拒绝过
        self.approve_lock = threading.Lock()
        self.ids = itertools.count(1)
        self._pose0 = None                    # 本次自主动作开始前的位姿
        self._state_cache = (0.0, None)
        self._stop_lock = threading.Lock()
        self._stop_client = None
        cfg.hooks = WebHooks(self)

    # ---------- 事件流 ----------
    def publish(self, ev):
        ev = dict(ev)
        with self.lock:
            ev["seq"] = next(self.seq)
            ev["wall"] = time.time()
            self.ring.append(ev)
            del self.ring[:-RING_MAX]
            subs = list(self.subs)
        for q in subs:
            try:
                q.put_nowait(ev)
            except queue.Full:
                pass

    def on_event(self, ev):
        """在事件进流之前顺手量一次"轮子真的动了多少"，和工具自报的话并排放。"""
        kind = ev.get("type")
        tool = ev.get("tool")
        if kind == "call" and tool in MOTION_TOOLS:
            self._pose0 = self.pose()
            ev["pose0"] = self._pose0
        elif kind == "result" and tool in MOTION_TOOLS:
            now = self.pose()
            if self._pose0 and now:
                dx, dy = now[0] - self._pose0[0], now[1] - self._pose0[1]
                dyaw = (now[2] - self._pose0[2] + 180.0) % 360.0 - 180.0
                ev["measured"] = {"delta": round((dx * dx + dy * dy) ** 0.5, 3),
                                  "dyaw": round(dyaw, 1), "pose": [round(v, 3) for v in now]}
            self._pose0 = None
        self.publish(ev)

    def subscribe(self):
        q = queue.Queue(maxsize=4096)
        with self.lock:
            self.subs.append(q)
            replay = list(self.ring[-SSE_REPLAY:])
        snap = {"type": "snapshot", "busy": self.busy, "turns": self.turns,
                "tools": [t["function"]["name"] for t in self.tools_spec],
                "pending": [self._approval_payload(r, a) for r, a in self.pending.items()],
                "events": replay}
        return q, snap

    def unsubscribe(self, q):
        with self.lock:
            if q in self.subs:
                self.subs.remove(q)

    # ---------- 人工确认 ----------
    def _approval_payload(self, rid, ap):
        return {"id": rid, "tool": ap["tool"], "args": ap["args"],
                "seconds": self.approve_timeout}

    def ask_approval(self, name, args):
        # 一次拒绝就是本轮的判决：模型被拒后很容易立刻再问一遍，
        # 不停重新发卡片等于拿人的耐心当可以通过的东西。重新下指令才会恢复询问。
        if self.refused:
            self.on_event({"type": "auto_denied", "tool": name, "args": args})
            return False
        with self.approve_lock:
            rid = "a%d" % next(self.ids)
            ap = {"tool": name, "args": args, "allow": False, "ev": threading.Event()}
            self.pending[rid] = ap
        self.on_event({"type": "approval", **self._approval_payload(rid, ap)})
        answered = ap["ev"].wait(self.approve_timeout)
        with self.approve_lock:
            self.pending.pop(rid, None)
            allow = bool(ap["allow"]) and answered
        if not allow:
            self.refused = True
        if not answered:
            self.on_event({"type": "approval_expired", "id": rid, "tool": name,
                           "args": args})
        else:
            self.on_event({"type": "approval_decided", "id": rid, "tool": name,
                           "args": args, "allow": allow})
        return allow

    def answer(self, rid, allow):
        with self.approve_lock:
            ap = self.pending.get(rid)
            if ap is None or ap["ev"].is_set():
                return False            # 已经有人答过 / 已超时：先来的一方说了算
            ap["allow"] = bool(allow)
            ap["ev"].set()
        return True

    # ---------- 控制台读数 ----------
    def get_json(self, path, timeout=4.0):
        try:
            with urllib.request.urlopen(self.console + path, timeout=timeout) as r:
                return json.loads(r.read().decode("utf-8", "replace"))
        except (urllib.error.URLError, OSError, ValueError):
            return None

    def state(self):
        t0, cached = self._state_cache
        if cached is not None and time.monotonic() - t0 < STATE_TTL:
            return cached
        st = self.get_json("/api/state")
        if st is not None:
            self._state_cache = (time.monotonic(), st)
        return st

    def pose(self):
        st = self.state()
        p = (st or {}).get("pose")
        if isinstance(p, list) and len(p) == 3:
            return [float(p[0]), float(p[1]), float(p[2])]
        return None

    # ---------- 回合 ----------
    def start_turn(self, text, confirm=False):
        if self.busy:
            raise Busy("上一条指令还在执行，先等它结束或点急停")
        self.busy = True
        self.refused = False          # 新的一句话 = 新的授权机会
        self.cfg.confirm = bool(confirm)   # 这一轮要不要人点头；一轮只跑一条，改它是安全的
        threading.Thread(target=self._run, args=(text,), daemon=True).start()

    def _run(self, text):
        try:
            agent.run_turn(self.cfg, self.mcp, self.tools_spec, self.name_map,
                           self.messages, text)
        except (agent.McpError, RuntimeError) as exc:
            self.on_event({"type": "error", "text": "%s: %s" % (type(exc).__name__, exc)})
        finally:
            self.busy = False
            self.turns += 1
            self.trim()
            self.on_event({"type": "turn_end", "turns": self.turns})

    def trim(self, cap=60):
        """上下文太长会让小模型变傻、让 Ollama 变慢：只保 system + 最近若干条。"""
        if len(self.messages) <= cap:
            return
        keep = self.messages[-cap:]
        while keep and keep[0]["role"] == "tool":
            keep.pop(0)          # 别从半个 tool_calls 配对中间切开
        self.messages[:] = [self.messages[0]] + keep
        self.on_event({"type": "context_trimmed", "n": len(self.messages)})

    def reset(self):
        if self.busy:
            raise Busy("正在执行，不能清空上下文")
        self.messages[:] = [self.messages[0]]
        self.on_event({"type": "context_cleared"})

    # ---------- 急停 ----------
    def stop(self):
        """两条路都发：控制台那条掐点动并置 nav_stop（新加的 Move 循环会看它），
        MCP 那条掐自主动作循环。任何一个失败都会照实说。"""
        out = {}
        try:
            req = urllib.request.Request(self.console + "/api/stop", data=b"{}",
                                         headers={"Content-Type": "application/json"},
                                         method="POST")
            with urllib.request.urlopen(req, timeout=4.0) as r:
                out["console"] = json.loads(r.read().decode("utf-8", "replace")).get("ok")
        except (urllib.error.URLError, OSError, ValueError) as exc:
            out["console"] = "失败: %s" % exc
        try:
            with self._stop_lock:
                if self._stop_client is None:
                    # 专用会话：主 MCP 客户端此刻可能正卡在一条 Move 里没返回
                    self._stop_client = agent.McpClient(self.cfg.mcp_url, timeout=15.0)
                    self._stop_client.connect()
                out["mcp"] = self._stop_client.call_tool("Stop", {})
        except (agent.McpError, OSError, ValueError) as exc:
            self._stop_client = None
            out["mcp"] = "失败: %s" % exc
        self.on_event({"type": "estop", "text": json.dumps(out, ensure_ascii=False)})
        return out


class WebHooks:
    """agent.run_turn 的网页版出口：事件进 SSE，确认变成页面上的一张卡片。"""

    def __init__(self, session):
        self.s = session

    def event(self, ev):
        self.s.on_event(ev)

    def confirm(self, name, args):
        return self.s.ask_approval(name, args)


class Handler(BaseHTTPRequestHandler):
    session = None       # 由 serve() 注入
    asr = None           # 由 main() 注入
    speaker = None       # 由 main() 注入
    server_version = "CarChat/1.0"
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass

    def handle(self):
        # 浏览器随时会半路扔掉一次请求（切后台、刷新、地图轮询被下一张顶掉）。
        # 这类断连会从"读请求行"或"写响应体"两处抛出来，socketserver 每次都糊一整页
        # traceback 进日志；急停和转写日志就在这页里，噪声得压掉（事件流那条已自行处理）。
        try:
            super().handle()
        except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
            pass

    def _same_origin(self):
        """这页面无鉴权，默认只监听 127.0.0.1；再挡一道跨站 POST，
        免得别的网页隔着浏览器往这台笔记本上发指令。急停不受此限制。"""
        origin = self.headers.get("Origin")
        if not origin:
            return True
        host = urllib.parse.urlparse(origin).netloc
        return host.lower() == (self.headers.get("Host") or "").lower()

    # ---- 小工具 ----
    def _headers(self, ctype, extra=None):
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)

    def _send(self, code, body, ctype="application/json; charset=utf-8", extra=None):
        data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self._headers(ctype, extra)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _proxy(self, path, ctype):
        try:
            with urllib.request.urlopen(self.session.console + path, timeout=8.0) as r:
                self._send(r.status, r.read(), ctype=r.headers.get("Content-Type") or ctype)
        except urllib.error.HTTPError as exc:
            self._send(exc.code, exc.read(),
                       ctype=exc.headers.get("Content-Type") or "text/plain; charset=utf-8")
        except (urllib.error.URLError, OSError) as exc:
            self._send(502, ("控制台不可用: %s" % exc).encode(),
                       ctype="text/plain; charset=utf-8")

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n).decode("utf-8", "replace") if n else ""
        try:
            return json.loads(raw) if raw.strip() else {}
        except ValueError:
            return None

    def _raw_body(self, limit):
        """把 Content-Length 这么多字节读进来，超过 limit 就丢掉这个请求。

        不能只信客户端报的头：报一个 2 GB 的头会把内存吃掉，所以按剩余配额读，
        读完还留着没读的部分直接断开连接（HTTP/1.1 下 keep-alive 会复用连接，
        留着脏字节会让下一个请求解析出错，所以这里必须关掉复用）。
        """
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0:
            return b"", "请求体是空的"
        if n > limit:
            self.close_connection = True
            return None, "音频太大（%d 字节），最长约 6 分钟" % n
        return self.rfile.read(n), None

    def _car_summary(self):
        st = self.session.state() or {}
        reloc = st.get("reloc") or {}
        return {"online": bool(st),
                "pose": st.get("pose"), "busy": st.get("busy"),
                "volts": st.get("volts"), "mapping": st.get("mapping"),
                "frozen": reloc.get("frozen"), "why": reloc.get("why"),
                "agree": reloc.get("agree"), "odom_age": st.get("odom_age"),
                "scan_age": st.get("scan_age"), "teleop": st.get("teleop")}

    # ---- 路由 ----
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path, q = parsed.path, urllib.parse.parse_qs(parsed.query)
        s = self.session
        if path in ("/", "/index.html"):
            try:
                with open(UI_FILE, "rb") as f:
                    html = f.read()
            except OSError as exc:
                return self._send(500, {"error": "读不到页面 %s: %s" % (UI_FILE, exc)})
            return self._send(200, html, "text/html; charset=utf-8")
        if path == "/events":
            return self.sse()
        if path == "/state":
            return self._send(200, {"chat": {"busy": s.busy, "turns": s.turns,
                                              "pending": [s._approval_payload(r, a)
                                                          for r, a in s.pending.items()]},
                                    "voice": {"asr": self.asr.info(), "tts": self.speaker.info()},
                                    "car": self._car_summary()})
        if path == "/history":
            with s.lock:
                events = list(s.ring[-RING_MAX:])
            return self._send(200, {"events": events, "busy": s.busy})
        if path == "/tools":
            return self._send(200, {"tools": [
                {"name": t["function"]["name"],
                 "description": t["function"]["description"].splitlines()[0][:160]}
                for t in s.tools_spec]})
        if path == "/map.png":
            want = ["half=%s" % (q.get("half", ["6.0"])[0]),
                    "center=%s" % (q.get("center", ["car"])[0]),
                    "lidar=%s" % (q.get("lidar", ["1"])[0])]
            return self._proxy("/api/map.png?" + "&".join(want), "image/png")
        if path == "/shot":
            return self._proxy("/api/shot", "image/jpeg")
        if path == "/img":
            return self.local_image(q.get("path", [""])[0])
        return self._send(404, {"error": "no such path: %s" % path})

    def do_POST(self):
        path = urllib.parse.urlparse(self.path).path
        if path == "/asr":
            # 语音走的是原始字节，不是 JSON，所以必须排在 _body() 之前
            if not self._same_origin():
                return self._send(403, {"error": "拒绝跨站请求"})
            data, err = self._raw_body(ASR_MAX_BYTES)
            if err:
                return self._send(400, {"error": err})
            res = self.asr.transcribe(data)
            # 每一次真的按过说话都留一行：转了几秒音频、听出来几个字、还是没听清。
            # 否则"语音到底通不通"只剩用户口头一句，事后无从核对（第一次真麦实测就吃了这个亏）。
            sys.stderr.write("[asr] %.1f KB -> %s\n" % (
                len(data) / 1024.0,
                ("「%s」%.2f s%s" % (
                    res.get("text"), res.get("secs"),
                    ("（原听成「%s」：%s）" % (res["heard"], "、".join(res["fixes"]))
                     if res.get("fixes") else ""))
                 if res.get("ok") else res.get("error"))))
            sys.stderr.flush()
            if res.get("ok"):
                return self._send(200, res)
            # 没装是 503（页面的语音状态行会显示"识别没装"），送错东西才是 400
            return self._send(503 if not self.asr.ready else 400, res)
        body = self._body()
        if body is None:
            return self._send(400, {"error": "请求体不是合法 JSON"})
        s = self.session
        try:
            if path == "/chat":
                if not self._same_origin():
                    return self._send(403, {"error": "拒绝跨站请求"})
                text = (body.get("text") or "").strip()
                if not text:
                    return self._send(400, {"error": "text 不能为空"})
                s.start_turn(text, bool(body.get("confirm")))
                return self._send(202, {"ok": True})
            if path == "/speak":
                if not self._same_origin():
                    return self._send(403, {"error": "拒绝跨站请求"})
                res = self.speaker.speak(body.get("text") or "")
                return self._send(200 if res.get("ok") else 400, res)
            if path == "/speak_stop":
                if not self._same_origin():
                    return self._send(403, {"error": "拒绝跨站请求"})
                return self._send(200, self.speaker.stop())
            if path == "/approval":
                if not self._same_origin():
                    return self._send(403, {"error": "拒绝跨站请求"})
                ok = s.answer(str(body.get("id") or ""), bool(body.get("allow")))
                return self._send(200 if ok else 409,
                                  {"ok": ok} if ok else {"error": "这条确认已经作废（超时或别人已答）"})
            if path == "/stop":
                out = s.stop()
                out["voice"] = self.speaker.stop().get("was")     # 车停了话也不该继续说
                return self._send(200, out)
            if path == "/reset":
                if not self._same_origin():
                    return self._send(403, {"error": "拒绝跨站请求"})
                s.reset()
                return self._send(200, {"ok": True})
        except Busy as exc:
            return self._send(409, {"error": str(exc)})
        return self._send(404, {"error": "no such path: %s" % path})

    def local_image(self, want):
        """只允许取 pc_side/cache 底下的图片，别把自己变成任意文件读取。
        Windows 的路径大小写不固定（c: 与 C: 都可能出现），所以用 normcase 比。"""
        if not want:
            return self._send(400, {"error": "path 必填"})
        real = os.path.normcase(os.path.realpath(want))
        base = os.path.normcase(CACHE_DIR)
        if real != base and not real.startswith(base + os.sep):
            return self._send(403, {"error": "只允许读 %s 下的图片" % CACHE_DIR})
        try:
            with open(real, "rb") as f:
                data = f.read()
        except OSError as exc:
            return self._send(404, {"error": "读不到: %s" % exc})
        self._send(200, data,
                   ctype="image/jpeg" if real.lower().endswith((".jpg", ".jpeg")) else "image/png")

    def sse(self):
        q, snap = self.session.subscribe()
        self.send_response(200)
        self._headers("text/event-stream; charset=utf-8",
                      {"Connection": "close", "X-Accel-Buffering": "no"})
        self.end_headers()
        try:
            self.wfile.write(("data: %s\n\n" % json.dumps(snap, ensure_ascii=False)).encode())
            self.wfile.flush()
            while True:
                try:
                    ev = q.get(timeout=8.0)
                    line = "data: %s\n\n" % json.dumps(ev, ensure_ascii=False)
                except queue.Empty:
                    line = ": ping\n\n"          # 心跳，免得代理把连接掐了
                self.wfile.write(line.encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            self.session.unsubscribe(q)


def build_parser():
    p = agent.build_argparser()
    p.description = "MicroROS-V2 网页聊天前端（纯标准库，:8020）"
    p.add_argument("--chat-port", type=int, default=8020)
    p.add_argument("--chat-host", default="127.0.0.1",
                   help="默认只听本机；这页面无鉴权，别直接开到公网/陌生网络")
    p.add_argument("--console-url", default=CONSOLE)
    p.add_argument("--approve-timeout", type=float, default=60.0)
    p.add_argument("--asr-exe", default=ASR_EXE, help="whisper-cli.exe 路径")
    p.add_argument("--asr-model", default=ASR_MODEL, help="ggml-*.bin 模型路径")
    p.add_argument("--asr-lang", default="zh", help="spoken language for whisper ('auto' to detect)")
    p.add_argument("--asr-threads", type=int, default=12)
    p.add_argument("--asr-timeout", type=float, default=25.0)
    p.add_argument("--asr-prompt", default=ASR_PROMPT,
                   help="塞给 whisper 的简体中文 initial prompt（压繁体、顺带纠同音错字）；给空串关掉")
    p.add_argument("--voice", default="", help="SAPI 嗓名，空=系统默认（本机装了 Microsoft Huihui Desktop 中文）")
    p.add_argument("--voice-chars", type=int, default=400, help="一句回复最多念多少字")
    p.add_argument("--no-voice", action="store_true", help="不启用朗读（页面勾选也无效）")
    return p


def main():
    cfg = build_parser().parse_args()
    if not cfg.mcp_url:
        if not cfg.car:
            print("缺少 MCP 地址：用 --mcp-url http://127.0.0.1:8000/mcp 或 --car <ip>")
            sys.exit(2)
        cfg.mcp_url = "http://%s:8000/mcp" % cfg.car
    mcp = agent.McpClient(cfg.mcp_url, timeout=cfg.mcp_timeout)
    print("连接 MCP 服务: %s" % cfg.mcp_url)
    try:
        mcp.connect()
        tools = mcp.list_tools()
    except agent.McpError as exc:
        print("[连接失败] %s\n"
              "检查：① 先起「_web_bridge.bat」（本机 :8000 的 MCP 服务 + :8010 控制台）"
              "② micro-ROS Agent 是否在 8090/udp ③ 车上电并连着同一个 WiFi" % exc)
        sys.exit(1)
    tools_spec, name_map = agent.mcp_tools_to_openai(tools)
    session = Session(cfg, mcp, tools_spec, name_map, console=cfg.console_url,
                      approve_timeout=cfg.approve_timeout)
    Handler.session = session
    Handler.asr = Asr(cfg.asr_exe, cfg.asr_model, cfg.asr_lang, cfg.asr_threads,
                      cfg.asr_timeout, cfg.asr_prompt)
    Handler.speaker = Speaker(voice=cfg.voice, max_chars=cfg.voice_chars)
    Handler.speaker.on = not cfg.no_voice
    httpd = ThreadingHTTPServer((cfg.chat_host, cfg.chat_port), Handler)
    print("大模型: %s @ %s%s" % (cfg.model, cfg.llm_base,
                                 "   视觉模型: " + cfg.vision_model if cfg.vision_model else ""))
    print("语音输入: %s" % ("whisper.cpp %s (%s, %d 线程)" % (
        os.path.basename(Handler.asr.model), Handler.asr.lang, Handler.asr.threads)
        if Handler.asr.ready else "缺 whisper-cli 或模型，页面的麦克风按钮会提示不可用"))
    print("语音输出: %s" % ("SAPI 嗓「%s」" % (Handler.speaker.voice or "系统默认")
                            if Handler.speaker.ready else "朗读不可用（缺 %s 或 --no-voice）" % SPEAK_PS1))
    if cfg.dry_run:
        print("[dry-run] 工具调用只打印，不会下发给车")
    print("车上 %d 个工具：%s" % (len(tools), " / ".join(name_map.values())))
    print("聊天页面: http://%s:%d/   控制台: %s   停止: Ctrl+C"
          % (cfg.chat_host, cfg.chat_port, cfg.console_url))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[退出]")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
