#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 40 句已知指令灌进**正在跑的** /asr，量当前这条链路的真实准确率。

为什么要它：`_voice_test.py` 只管"有没有坏"，而"同音纠正表该收哪些词"必须由当前配置
（base 档 + ASR_PROMPT + 繁简表 + fix_command）的实际输出说了算。换模型档、改 prompt、
动表之后都该重跑一遍 —— 实测就抓到过一件事：加了领域 prompt 之后 建築/糧米 不再错，
但"扫描一下"从"少描移下"变成了"搜妙移下"，**错字会换地方**，照旧表纠就漏。

只读：调 /asr（不改位姿、不动电机、不写 bias），跑完把还剩哪几句不对打印出来。

跑法：`python C:\\microros-agent\\_asr_audit.py`（要先开着聊天页服务）
"""
import json
import os
import re
import sys
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import car_chat as cc  # noqa: E402

CORPUS = os.path.join(HERE, "asr", "test", "corpus")
ASR_URL = "http://127.0.0.1:8020/asr"
PUNCT = str.maketrans("", "", "。，,？?、！! \n")
DIGITS = {c: i for i, c in enumerate("零一二三四五六七八九")}


def cn_run(run):
    """把"五十 / 一百八十 / 十二"这类数量词换成阿拉伯数字。

    只为对账：whisper 常把"向前五十厘米"写成"向前50厘米"，**这不是听错**，
    拿原文比就会把这种算成错字，表也就跟着加错东西。用户的文本一律不改。
    """
    total = num = 0
    for ch in run:
        if ch in DIGITS:
            num = DIGITS[ch]
        elif ch in "十百千":
            total += (num or 1) * {"十": 10, "百": 100, "千": 1000}[ch]
            num = 0
        else:
            return run
    return str(total + num)


def norm(s):
    s = s.translate(PUNCT)
    return re.sub(r"[零一二三四五六七八九十百千]+", lambda m: cn_run(m.group()), s)


def main():
    rows_path = os.path.join(CORPUS, "rows.json")
    if not os.path.isfile(rows_path):
        # 语料不随离线包发（40 句 wav 会把包撑大），所以新机器上这里必须说清楚缺的是什么
        print("没有测试语料 %s —— 这 40 句不随离线包发。"
              "纠正表照样能用，只是换不了这张表；要复现它得从开发机拷 asr\\test\\corpus。" % rows_path)
        return 1
    rows = json.load(open(rows_path, encoding="utf-8"))
    bad, fixed_ok = [], []
    for i, (want, old_heard) in enumerate(rows):
        path = os.path.join(CORPUS, "s%d.wav" % i)
        if not os.path.isfile(path):
            print("跳过 %d：没有 %s" % (i, path))
            continue
        req = urllib.request.Request(
            ASR_URL, data=open(path, "rb").read(),
            headers={"Content-Type": "application/octet-stream",
                     "Origin": "http://127.0.0.1:8020"})
        try:
            res = json.load(urllib.request.urlopen(req, timeout=40))
        except Exception as exc:
            print("第 %d 句请求失败：%s（聊天服务开着吗？）" % (i, exc))
            return 1
        if not res.get("ok"):
            bad.append((i, want, "(没听清：%s)" % res.get("error"), ""))
            continue
        got = res["text"]
        if norm(got) == norm(want):
            if res.get("fixes"):
                fixed_ok.append((i, want, res["heard"], "、".join(res["fixes"])))
        else:
            bad.append((i, want, got, "、".join(res.get("fixes") or [])))
        print("  %2d %-16s -> %s%s" % (i, want, got,
                                       "   [纠了 %s]" % "、".join(res.get("fixes") or [])
                                       if res.get("fixes") else ""))
    print("\n本来就对 %d 句；靠纠正表救回来 %d 句；剩下不对 %d 句" % (
        len(rows) - len(bad) - len(fixed_ok), len(fixed_ok), len(bad)))
    for i, want, heard, hits in fixed_ok:
        print("  救回来：s%d 想要「%s」原听成「%s」（%s）" % (i, want, heard, hits))
    for i, want, got, hits in bad:
        print("  还不对：s%d 想要「%s」听到「%s」%s" % (i, want, got,
                                                    "（纠了 %s 仍不够）" % hits if hits else ""))
    print("\n把上面这些错字加进 car_chat.py 的 ASR_FIX（绑短语），再跑 _voice_test.py 第 8 段复核。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
