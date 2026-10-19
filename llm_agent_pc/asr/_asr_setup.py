#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把语音这条路要用的两样东西取回本机：whisper.cpp 的 Windows 预编译 + ggml 中文模型。

    python C:\\microros-agent\\asr\\_asr_setup.py               # 只要默认那份（whisper-bin-x64 + base）
    python C:\\microros-agent\\asr\\_asr_setup.py --models base small
    python C:\\microros-agent\\asr\\_asr_setup.py --list        # 只看有哪些档，不下

两个来源都不是"顺手能通"的那种，路线是实测出来的：
  * github.com 的 HTTPS 在这台机器上是连不上的（SNI 层被挡），但 **api.github.com 可达**，
    而且 `GET /repos/<o>/<r>/releases/assets/<id>` 带 `Accept: application/octet-stream`
    会 302 到 release-assets.githubusercontent.com —— 那个域名能连，还答 Range（206）。
    实测 ~200 KB/s，8.5 MB 的 zip 一分钟内到。
  * 模型走 hf-mirror.com：`/ggerganov/whisper.cpp/resolve/main/<file>` 302 到 CDN，也能 Range。
    （huggingface.co 本身在这台机器上超时。）

下载都带断点续传 + 重试，因为这两个源都可能中途断，一次失败不代表源坏了。
"""

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
BIN_DIR = os.path.join(HERE, "bin")
MODEL_DIR = os.path.join(HERE, "models")
ASSET = "whisper-bin-x64.zip"                 # CPU 版预编译；CUDA 那两个档 650 MB 且没有 sm_120
RELEASE = ("ggml-org", "whisper.cpp")
HF = "https://hf-mirror.com/ggerganov/whisper.cpp/resolve/main/%s"
UA = "microros-asr-setup"


def get(url, headers=None, timeout=60.0):
    req = urllib.request.Request(url, headers={"User-Agent": UA, **(headers or {})})
    return urllib.request.urlopen(req, timeout=timeout)


def asset_id(name):
    """从最近的几个 release 里找到指定资产名的数字 id（资产列表才是"有没有预编译"的证据）。"""
    with get("https://api.github.com/repos/%s/%s/releases?per_page=8" % RELEASE,
             {"Accept": "application/vnd.github+json"}) as r:
        rels = json.loads(r.read().decode("utf-8", "replace"))
    for rel in rels:
        for a in rel.get("assets") or []:
            if a["name"] == name:
                print("找到 %s  来自 %s  %.1f MB" % (name, rel["tag_name"], a["size"] / 1048576.0))
                return a["id"], a["size"]
    raise SystemExit("最近的 release 里没有 %s；上面列出的资产名就是可选范围" % name)


def fetch(url, dest, headers=None, tries=12):
    """带 Range 续传的下载：源会断，所以每次重试都从已落盘的字节接着要。"""
    have = os.path.getsize(dest) if os.path.exists(dest) else 0
    for n in range(tries):
        h = {"Range": "bytes=%d-" % have} if have else {}
        h.update(headers or {})
        try:
            with get(url, h) as r:
                total = have + int(r.headers.get("Content-Length") or 0)
                mode = "wb" if have == 0 else "ab"
                t0, last, buf = time.time(), have, b""
                with open(dest, mode) as fh:
                    while True:
                        buf = r.read(65536)
                        if not buf:
                            break
                        fh.write(buf)
                        have += len(buf)
                        if have - last > 4 * 1048576:
                            print("\r  %6.1f/%s MB  %.0f KB/s" % (
                                have / 1048576.0, total / 1048576.0 if total else "?",
                                (have - last) / 1024.0 / max(time.time() - t0, 1e-6)), end="")
                            sys.stdout.flush()
                            t0, last = time.time(), have
                print("\r  落地 %s  %.1f MB          " % (dest, have / 1048576.0))
                if total and have < total:
                    raise OSError("断了，还差 %d 字节" % (total - have))
                return have
        except (urllib.error.URLError, OSError, ValueError) as exc:
            print("\n  第 %d 次没成：%s（%s），从 %d 字节续" % (n + 1, type(exc).__name__, exc, have))
            time.sleep(min(10, 2 + n))
    raise SystemExit("下不动：%s" % url)


def get_bin(force=False):
    exe = os.path.join(BIN_DIR, "Release", "whisper-cli.exe")
    if os.path.isfile(exe) and not force:
        print("whisper-cli 已就位：%s" % exe)
        return exe
    aid, size = asset_id(ASSET)
    url = "https://api.github.com/repos/%s/%s/releases/assets/%d" % (RELEASE[0], RELEASE[1], aid)
    zip_path = os.path.join(HERE, ASSET)
    if not os.path.exists(zip_path) or os.path.getsize(zip_path) != size:
        fetch(url, zip_path, {"Accept": "application/octet-stream"})
    with zipfile.ZipFile(zip_path) as z:
        z.extractall(BIN_DIR)
    os.remove(zip_path)
    if not os.path.isfile(exe):
        raise SystemExit("解压后没看到 %s" % exe)
    print("whisper-cli 解压到：%s" % exe)
    return exe


def get_models(names):
    os.makedirs(MODEL_DIR, exist_ok=True)
    out = []
    for n in names:
        fn = "ggml-%s.bin" % n
        dest = os.path.join(MODEL_DIR, fn)
        if os.path.isfile(dest) and os.path.getsize(dest) > 1048576:
            print("模型已在：%s（%.1f MB）" % (dest, os.path.getsize(dest) / 1048576.0))
        else:
            fetch(HF % fn, dest)
        out.append(dest)
    return out


MODELS = ["tiny", "base", "small", "medium", "medium-q5_0", "large-v3-turbo-q5_0"]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--models", nargs="*", default=["base"], choices=MODELS,
                   help="要哪些 ggml 模型；实测本机 CPU：base 每条 0.5-1.2 s、small 1.7-3.6 s、"
                        "medium-q5_0 7-8 s 且并没有更准")
    p.add_argument("--skip-bin", action="store_true")
    p.add_argument("--list", action="store_true", dest="list_only")
    p.add_argument("--force", action="store_true")
    a = p.parse_args()
    # whisper.cpp 的 Windows 版按 ANSI 码页读 argv，路径带中文时 exe 和模型根本打不开
    # （实测退 127 / 0xC0000409，看着像程序坏了其实是装错了地方），所以装之前就拦。
    if any(ord(c) > 127 for c in HERE) and not a.list_only:
        print("这里的路径带中文 ——\n  %s\nwhisper 读不了这种路径，请把整个 asr 目录放到纯英文路径下"
              "（例如 C:\\microros-agent\\asr）再跑这个脚本。" % HERE)
        return
    if a.list_only:
        with get("https://hf-mirror.com/api/models/ggerganov/whisper.cpp/tree/main") as r:
            for f in json.loads(r.read().decode("utf-8", "replace")):
                if f["path"].endswith(".bin"):
                    print("%-38s %7.1f MB" % (f["path"], f.get("size", 0) / 1048576.0))
        return
    if not a.skip_bin:
        get_bin(a.force)
    for m in get_models(a.models):
        print("模型：%s" % m)
    print("\n启动聊天服务时不用改参数（默认就指这里）；要换档用 --asr-model 指到上面某个路径。")


if __name__ == "__main__":
    main()
