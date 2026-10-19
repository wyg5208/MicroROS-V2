#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""语音这条路（whisper.cpp 转写 + SAPI 朗读）的离线验收，不需要车在线、不出声。

跑法：`python C:\\microros-agent\\_voice_test.py`
（第 6 段会真的出声几句，介意噪音就跳过它。）

八段：
0. 手抄的繁简字对表自身（长度偶数、无自映射、无"繁体同时当简体"、简体原样不动）；
1. `Speaker.clip` 的字数闸与"没有 _speak.ps1 就说不可用"；
2. `Asr` 的三道输入门：太短、不是 WAV、太大 —— 这些都必须**不进子进程**就退回；
3. 用 SAPI 现场合成一句中文指令写成 WAV（`-Out`，不出声），交给真 whisper-cli 转写，
   看回来的字是不是那条指令（这是整条链路唯一"证明确实听懂了"的一段）；
4. 起一个临时 HTTP 服务走 /asr、/speak_stop、/state：验原始字节的读取、失败时的状态码，
   以及缺模型时页面能知道（ready=False）；
5. 朗读进程掐得掉（念长句途中 stop，进程应立刻没了）；
6. "没出声"要能看出来：脚本秒退但 rc=0（相当于接不到出声设备）必须被记成可疑，
   而真念一句长话不能被误报 —— 这条是补上"日志安静＝成功"那个坑的；
7. 装到带中文的路径要当场认出来：whisper 的 Windows 版按 ANSI 码页读 argv，
   路径里有中文时直接崩（离线包实测退 127 / 0xC0000409），必须回"挪到纯英文路径"
   而不是让人以为模型坏了；
8. 同音纠正 `fix_command`：表里的错字来自这份语料的真实听写结果，所以两头都归它管 ——
   40 句**正确原文**过一遍必须原样出来（不许改坏），声称能纠的必须纠到那句原话。

第 3 段要 whisper 的 exe 与模型都在，缺了会 SKIP 而不是 FAIL —— 别人没跑 _asr_setup.py 时
这套测试仍然应该能验前面几段里不依赖模型的部分。
"""
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import car_chat as cc  # noqa: E402

TMP = os.path.join(HERE, "asr", "test")
os.makedirs(TMP, exist_ok=True)
SAY = "前进一米然后右转四十五度"
fails = []


def check(name, got, want):
    ok = got == want
    print("%-58s %s  (got %r, want %r)" % (name, "PASS" if ok else "FAIL", got, want))
    if not ok:
        fails.append(name)


def post(path, data, ctype="application/json", port=8123):
    body = data if isinstance(data, bytes) else json.dumps(data).encode()
    req = urllib.request.Request("http://127.0.0.1:%d%s" % (port, path), data=body,
                                 headers={"Content-Type": ctype, "Origin": "http://127.0.0.1:%d" % port})
    try:
        with urllib.request.urlopen(req, timeout=60.0) as r:
            return r.status, json.loads(r.read().decode("utf-8", "replace"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8", "replace") or "{}")


def sapi_to_wav(text, dest):
    """把话写成 WAV（不经过喇叭），用来当"耳朵听到的东西"。"""
    txt = dest + ".txt"
    with open(txt, "w", encoding="utf-8") as fh:
        fh.write(text)
    r = subprocess.run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
                        "-File", cc.SPEAK_PS1, "-Path", txt, "-Voice", "Microsoft Huihui Desktop",
                        "-Out", dest], capture_output=True, timeout=60.0)
    os.remove(txt)
    if r.returncode != 0 or not os.path.isfile(dest):
        raise SystemExit("SAPI 合成失败，第 3/4 段没法验：\n%s" % r.stderr.decode("utf-8", "replace"))
    return dest


# ---- 0. 繁简字对表自身 ------------------------------------------------------
print("== 0. 繁体字对表 ==")
# 手抄表的三类错法都能被这几个断言抓住：抄漏一个字（奇数长度）、一对写成同一个字、
# 方向写反（把简体当键）—— 第三种最阴，因为它会把对的句子改坏。
check("长度是偶数（没有抄漏半个字对）", len(cc.ASR_T2S_PAIRS) % 2, 0)
check("没有自映射", [cc.ASR_T2S_PAIRS[i] for i in range(0, len(cc.ASR_T2S_PAIRS) - 1, 2)
                     if cc.ASR_T2S_PAIRS[i] == cc.ASR_T2S_PAIRS[i + 1]], [])
check("方向可疑的字（繁体同时当了简体）", sorted(set(cc._T2S) & set(cc._T2S.values())), [])
check("前进/右转的繁体在表里", [cc.to_simplified("前進"), cc.to_simplified("右轉")], ["前进", "右转"])
check("简体原样不动", cc.to_simplified("向前五十厘米，原地左转三十度"), "向前五十厘米，原地左转三十度")

# ---- 1. 朗读的闸 -------------------------------------------------------------
print("\n== 1. Speaker ==")
check("长句截到上限", cc.Speaker.clip("字" * 900, 400).count("字"), 400)
check("空白折成一句", cc.Speaker.clip("  念  着\n 很 顺  ", 400), "念 着 很 顺")
check("没有 _speak.ps1 就报告不可用", cc.Speaker(ps1=os.path.join(HERE, "nope.ps1")).ready, False)
sp = cc.Speaker(voice="Microsoft Huihui Desktop")
check("嗓名进 /state 的 info", sp.info()["voice"], "Microsoft Huihui Desktop")
check("空文本不念", sp.speak("   ").get("error"), "没有要念的字")

# ---- 2. 转写的三道门（都不许起子进程）--------------------------------------
print("\n== 2. Asr 输入门 ==")
asr = cc.Asr()
check("whisper 就位", asr.ready, True)
check("太短直接退", "太短" in asr.transcribe(b"RIFF" + b"x" * 20).get("error", ""), True)
check("不是 WAV 直接退", "不是 WAV" in asr.transcribe(b"MP3fake" * 20).get("error", ""), True)
check("太大直接退", "太大" in asr.transcribe(b"RIFF" + b"\0" * cc.ASR_MAX_BYTES).get("error", ""), True)
missing = cc.Asr(exe=os.path.join(HERE, "nope.exe"))
check("缺 whisper-cli 时说不装", "whisper" in missing.transcribe(b"RIFF" + b"\0" * 100).get("error", ""), True)

# ---- 3. 真转写：SAPI 说的那句要能听回来 -------------------------------------
print("\n== 3. 端到端听写 ==")
wav = sapi_to_wav(SAY, os.path.join(TMP, "utt-%d.wav" % int(time.time())))
res = asr.transcribe(open(wav, "rb").read())
text = res.get("text", "")
print("   听到: %r  (%.2f s)" % (text, res.get("secs", 0)))
check("转写成功", res.get("ok"), True)
check("有'前进'", "前进" in text, True)
check("有'1'或'一'（数字写法不强制）", ("1" in text) or ("一" in text), True)
check("有'米'", "米" in text, True)
check("有'右转'（繁体已由字对表抹平）", "右转" in text, True)
check("输出里没有表内的繁体字", sorted({c for c in text if c in cc._T2S}), [])
check("单条延迟 8 s 以内（实测 base+prompt 约 1.8 s）", res.get("secs", 99) < 8.0, True)
# 注音式的幻觉输出不该进输入框：那是"没听清"，不是"听到一句奇怪的话"（实测"停车"曾出 ㄇㄇㄇ）
check("注音乱码不算合法文本", cc.looks_like_speech(cc.to_simplified("ㄅㄅㄇㄇ")), False)
check("汉字/数字算合法文本", [cc.looks_like_speech(s) for s in ("前进一米", "停 20 米", "。？！")],
      [True, True, False])

# ---- 4. 走 HTTP：原始字节、状态码、/state 里看得见语音状态 -------------------
print("\n== 4. HTTP 路由 ==")


class FakeSession:
    busy, turns = False, 0
    pending = {}
    console = "http://127.0.0.1:1"

    def state(self):
        return None

    def stop(self):
        return {"console": "stub", "mcp": "stub"}


cc.Handler.session = FakeSession()
cc.Handler.asr = asr
cc.Handler.speaker = cc.Speaker()
httpd = ThreadingHTTPServer(("127.0.0.1", 8123), cc.Handler)
threading.Thread(target=httpd.serve_forever, daemon=True).start()
code, j = post("/asr", open(wav, "rb").read(), "application/octet-stream")
check("/asr 200", code, 200)
check("/asr 带字", ("米" in j.get("text", "")), True)
code, j = post("/asr", b"not a wave" * 40, "application/octet-stream")
check("/asr 非 WAV -> 400", code, 400)
code, j = post("/asr", b"RIFF" + b"x" * 300, "application/octet-stream")
check("/asr 空音频段 -> 4xx", code in (400, 422), True)
cc.Handler.asr = cc.Asr(exe=os.path.join(HERE, "nope.exe"))
code, j = post("/asr", open(wav, "rb").read(), "application/octet-stream")
check("/asr 没装 whisper -> 503", code, 503)
cc.Handler.asr = asr
with urllib.request.urlopen("http://127.0.0.1:8123/state", timeout=10.0) as r:
    st = json.loads(r.read().decode("utf-8", "replace"))
check("/state 报告 asr ready", st["voice"]["asr"]["ready"], True)
check("/state 报告默认模型", st["voice"]["asr"]["model"], "ggml-base.bin")
httpd.shutdown()

# ---- 5. 念长句途中能掐掉 ----------------------------------------------------
print("\n== 5. 朗读掐得掉 ==")
long_text = ("这是一段用来验证朗读可以被立刻打断的中文句子，" * 12)
r = cc.Handler.speaker.speak(long_text)
check("起得来", r.get("ok"), True)
time.sleep(1.0)
check("正在念", cc.Handler.speaker.proc is not None, True)
t0 = time.time()
cc.Handler.speaker.stop()
check("stop 之后没有活口", cc.Handler.speaker.proc is None, True)
check("掐掉只花 2 s 以内", time.time() - t0 < 2.0, True)
check("自己掐掉的这句不该报错", cc.Handler.speaker.last, "")
os.remove(wav)

# 第 6 段验"把安静当成功"这个坑：脚本秒退但 rc=0（等于接不到出声设备）时必须被点出来。
# 反过来，真念一句长话要十几秒，不能被误报。
print("\n== 6. 没出声要能看出来 ==")
stub = os.path.join(TMP, "_stub_quiet.ps1")
with open(stub, "w", encoding="utf-8") as fh:
    fh.write("exit 0\n")
fake = cc.Speaker(ps1=stub)
fake.speak("这句话脚本一毫秒就退了，应当判定成没出声。")
t0 = time.time()
while not fake.last and time.time() - t0 < 10:
    time.sleep(0.1)
check("秒退会被记成一条", fake.last != "", True)
check("并且点出「没出声」", "没出声" in fake.last, True)
check("状态里也带得上", "没出声" in fake.info()["last"], True)
real = cc.Handler.speaker
real.speak(long_text[:24])
t0 = time.time()
while real.proc is not None and time.time() - t0 < 40:
    time.sleep(0.5)
check("真念完了会留一条记录", real.last != "", True)
check("正常念的不会被误报成没出声", "没出声" in real.last, False)
print("   实测：%s" % real.last)
os.remove(stub)

# 第 7 段：whisper 的 Windows 版按 ANSI 码页读 argv，路径带中文会直接崩
# （离线包实测：Git Bash 退 127、Python 退 0xC0000409），所以要把装错地方说成装错地方。
print("\n== 7. 中文路径要当场认出来 ==")
zh = os.path.join(TMP, "语音")
os.makedirs(zh, exist_ok=True)
fake_exe = os.path.join(zh, "whisper-cli.exe")
open(fake_exe, "wb").close()          # 文件在，只是地方不对 —— 别让 ready 先替它说话
bad = cc.Asr(exe=fake_exe, model=cc.ASR_MODEL)
check("带中文的 exe 路径会被点出来", bad.path_error() is not None, True)
check("并且给的是挪路径的办法", "纯英文路径" in bad.path_error(), True)
check("转写会直接回这条错误而不是等崩",
      "纯英文路径" in bad.transcribe(b"RIFF" + b"x" * 300).get("error", ""), True)
check("状态里也带得上", bool(bad.info()["path_error"]), True)
check("本机默认安装位置是干净的",
      cc.non_ascii_path(cc.ASR_EXE, cc.ASR_MODEL, cc.ASR_TMP), [])
check("页面能知道这条（真 asr 没有路径问题）", cc.Handler.asr.info()["path_error"], None)
os.remove(fake_exe)

# 第 8 段：同音纠正。裁判用的就是选档时那 40 句实测语料 —— 表里的错字都是从这些真实
# 听写结果里挑出来的，所以"纠得对"和"没改坏"两头都得由同一份数据说话。
print("\n== 8. 同音纠正：纠得对，也不许改坏 ==")
PUNCT = str.maketrans("", "", "。，,？?、！! \n")
rows_path = os.path.join(TMP, "corpus", "rows.json")
if not os.path.isfile(rows_path):
    print("   SKIP：没有 40 句语料（%s）" % rows_path)
else:
    rows = json.load(open(rows_path, encoding="utf-8"))

    def norm(s):
        return s.translate(PUNCT)

    def fixed(s):
        return cc.fix_command(cc.to_simplified(s))[0]

    damaged = [(w, fixed(w)) for w, _ in rows if fixed(w) != cc.to_simplified(w)]
    check("%d 句正确原文一个字都不许动" % len(rows), damaged, [])
    # 声称能纠的，纠完就得等于当时说的那句（引号里是语料下标，跑的时候能对着 rows.json 查）
    FIX_EQ = (
        ("停在建築。", "停在建图", "17 建筑→建图"),
        ("打開射向頭。", "打开摄像头", "18 射向头→摄像头"),
        ("關閉射向頭。", "关闭摄像头", "19 同上"),
        ("善停", "暂停", "27 善停→暂停"),
        ("輕空上下文。", "清空上下文", "29 轻空→清空"),
        ("前進糧米", "前进两米", "32 粮米→两米"),
        ("看看有没有驾驻。", "看看有没有卡住", "39 驾驻→卡住"),
        ("去左邊納面牆看看。", "去左边那面墙看看", "13 纳面→那面"),
        ("延强边走一圈", "沿墙边走一圈", "15 强边→墙边"),
        ("向前50里米", "向前50厘米", "0 里米→厘米"),
        ("网前走依米", "往前走1米", "用户实测报回来的那句"),
        ("又转90度", "右转90度", "又→右"),
        ("听下", "停下", "听→停"),
        ("言强边走一圈", "沿墙边走一圈", "两条规则要连着走完（实测只走一遍会停在言墙边）"),
        ("现在你在地图等什么位置?", "现在你在地图的什么位置", "等→的"),
        ("观比射向头", "关闭摄像头", "两条规则各纠一半"),
    )
    for heard, want, why in FIX_EQ:
        check("纠到位：%s" % why, norm(fixed(heard)), norm(want))
    check("半句里的两处都能纠（少描移下→扫描一下）",
          norm(fixed("少描移下環境")), norm("扫描一下环境"))
    once = [fixed(h) for _, h in rows]
    check("纠两遍和纠一遍一样", [cc.fix_command(x)[0] for x in once], once)
    check("命中时会把原话和改了哪几处一起交出去",
          cc.fix_command("网前走依米")[1], ["网前→往前", "依米→1米"])
    check("没命中就绝不留痕", cc.fix_command("往前走两步")[1], [])
    check("刻意不纠的两处保持原样（倾斜是真词、《战体》是幻觉）",
          [fixed("车是不是倾斜了"), fixed("《战体》")], ["车是不是倾斜了", "《战体》"])

print("\n%s  (%d 处失败)" % ("全部 PASS" if not fails else "有 FAIL: " + ", ".join(fails), len(fails)))
sys.exit(1 if fails else 0)
