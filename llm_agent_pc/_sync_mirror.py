"""把权威目录 C:\\microros-agent 同步到 G 盘的两个镜像：
  llm_agent_pc/            —— 代码镜像（windows_side 里是精选的一批，不搬 cache/日志）
  0.离线软件包/07_PC侧代码/ —— 离线交付镜像（额外要 asr/bin 与 ggml-base.bin，断网也能用语音）
文档三份同时落到仓库根（G:\\baidunetdisk\\Microros-V2\\）。

跑法：python C:\\microros-agent\\_sync_mirror.py         # 只搬不一致的
      python C:\\microros-agent\\_sync_mirror.py --check  # 只报告，不写盘
      python C:\\microros-agent\\_sync_mirror.py --prune   # 再清掉离线包里多出来的语音文件
搬完逐个比 md5；对不上就非零退出，绝不把"cp 没报错"当成同步成功。
**目标是新的就不搬**（比源新 = 镜像那一侧有人撰稿过，见 job() 里的事故说明）：报"冲突"、跳过它、
退出码非零，让人先决定往哪个方向合。要复测这个门：`python _sync_conflict_test.py`（全离线）。
"""
import hashlib
import os
import shutil
import sys

SRC = r"C:\microros-agent"
G = r"G:\baidunetdisk\Microros-V2"
PC = os.path.join(SRC, "pc_side")
VC = os.path.join(SRC, "vehicle_config")
MIRROR_CODE = os.path.join(G, "llm_agent_pc")
MIRROR_WIN = os.path.join(MIRROR_CODE, "windows_side")
MIRROR_OFF = os.path.join(G, "0.离线软件包", "07_PC侧代码")
DOCS = ["README.md", "开发过程实录.md", "用户操作手册.md"]
# windows_side 里那几个不在 pc_side 下的东西，来源要单独指
OVERRIDE = {
    "启动Agent.bat": os.path.join(PC, "启动Agent.bat"),
    "build_agent.bat": os.path.join(PC, "build_agent.bat"),
    "fix_pixi_paths.bat": os.path.join(SRC, "fix_pixi_paths.bat"),
    "mcp_probe.py": os.path.join(PC, "mcp_probe.py"),
}
TOP = ["agent.py", "car_chat.py", "car_chat_ui.html", "_speak.ps1", "_voice_test.py",
       "_asr_audit.py", "_relaunch_chat.ps1", "启动智能体.bat", "启动聊天页面.bat",
       "_sync_mirror.py", "_sync_conflict_test.py",
       "car_cron.py", "cron_tasks.json", "安装定时任务.bat", "卸载定时任务.bat"]
def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for blk in iter(lambda: f.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


def job(src, dst):
    if not os.path.isfile(src):
        return ("缺源", src)
    if os.path.isfile(dst):
        if md5(src) == md5(dst):
            return ("已一致", dst)
        # 2026-10-19 实测踩过：《用户操作手册.md》的编辑只写在 G: 那一侧，这一句把它静默盖回了 HEAD，
        # 73 行新内容没了、git status 里连痕迹都不留（因为它"同步成功"了）。copy2 会把源的 mtime
        # 带到目标，所以真同步过的文件两边 mtime 相同 —— **目标比源新**只可能是有人在镜像那侧动了笔，
        # 那不是"源落后"，是要人判断的冲突。宁可报冲突不搬，也不能搬掉别人的撰稿。
        try:
            if os.path.getmtime(dst) > os.path.getmtime(src) + 1.0:
                return ("冲突", dst)
        except OSError:
            pass
    return ("要搬", "%s -> %s" % (src, dst))


def extra_files(moves):
    """离线交付包里，凡是不在待搬清单里的文件都算"多出来的"。

    实测踩过这个坑：开发时顺手把 ggml-small / ggml-medium 和测试语料拷进了
    07_PC侧代码/asr，一个交付包被撑到 1 GB（文档写的却是 154 MB），而这两个档
    实测就是不该随包发的（base 够用，medium 慢 6 倍不更准）。包里的文件必须
    能逐条对上交付清单，不然 verify 只会把它们一起登记成"该有"。
    只查 asr 这一处：包里其它目录是照镜像现有清单逐个刷新的（清单 == 目录本身），
    拿 moves 去比它们等于自己咬自己。
    """
    scope = os.path.normcase(os.path.abspath(os.path.join(MIRROR_OFF, "asr")))
    want = {os.path.normcase(os.path.abspath(d)) for _, d in moves
            if os.path.normcase(os.path.abspath(d)).startswith(scope)}
    out = []
    for root, dirs, files in os.walk(os.path.join(MIRROR_OFF, "asr")):
        dirs[:] = [x for x in dirs if x != "__pycache__"]
        for name in files:
            p = os.path.abspath(os.path.join(root, name))
            if os.path.normcase(p) not in want:
                out.append(p)
    return sorted(out)


def main(check_only):
    moves = []
    # 1) 聊天侧文件进两个镜像根；文档进仓库根与离线包（llm_agent_pc 里原本不放文档）
    for name in TOP:
        s = os.path.join(SRC, name)
        for d in (os.path.join(MIRROR_CODE, name), os.path.join(MIRROR_OFF, name)):
            moves.append((s, d))
    for name in DOCS:
        s = os.path.join(SRC, name)
        for d in (os.path.join(G, name), os.path.join(MIRROR_OFF, name)):
            moves.append((s, d))
    # 2) windows_side / pc_side：按镜像现有的清单逐个刷新（保持精选，不往里倒日志与快照），
    #    再补上"新写的验收脚本 / 新写的重启脚本"这类该进镜像而从没有过的文件
    NEW_TESTS = ("_stall_test.py", "_beep_unit_test.py", "_fit_test.py", "_relaunch_bridge.ps1", "_clamp_test.py")
    for mirror in (MIRROR_WIN, os.path.join(MIRROR_OFF, "pc_side")):
        if not os.path.isdir(mirror):
            continue
        for name in sorted(os.listdir(mirror)):
            p = os.path.join(mirror, name)
            if os.path.isdir(p):
                continue
            s = OVERRIDE.get(name) or os.path.join(PC, name)
            if not os.path.isfile(s):
                s = os.path.join(SRC, name)
            if os.path.isfile(s):
                moves.append((s, p))
        for name in NEW_TESTS:
            s = os.path.join(PC, name)
            if os.path.isfile(s):
                moves.append((s, os.path.join(mirror, name)))
    # 3) vehicle_config 两个镜像各一份（__pycache__ 不进镜像）
    for mirror in (os.path.join(MIRROR_WIN, "vehicle_config"), os.path.join(MIRROR_OFF, "vehicle_config")):
        for name in os.listdir(VC):
            s = os.path.join(VC, name)
            if os.path.isfile(s):
                moves.append((s, os.path.join(mirror, name)))
    # 4) 语音：取件脚本进两个镜像；whisper 程序与 base 模型只进离线包。
    #    只带 whisper-cli 真会加载的那几个文件 —— zip 里还有 parakeet/vad/stream/测试用的一堆 exe，
    #    带上只是把 23 MB 的无关物塞进交付包。ggml-cpu-* 要全带：运行时会按 CPU  ISA 挑一个。
    #    注意：离线包目录名带中文，whisper 在**这里原地跑不起来**（它按 ANSI 码页读 argv，
    #    实测退 127 / 崩 0xC0000409）。这是随包交付、靠 2.安装到本机.bat 拷到 C:\microros-agent
    #    之后才用，不是搬错了地方 —— 别再"顺手"把 asr 从离线包里摘掉。
    for mirror in (MIRROR_CODE, MIRROR_OFF):
        moves.append((os.path.join(SRC, "asr", "_asr_setup.py"), os.path.join(mirror, "asr", "_asr_setup.py")))
    moves.append((os.path.join(SRC, "asr", "models", "ggml-base.bin"),
                  os.path.join(MIRROR_OFF, "asr", "models", "ggml-base.bin")))
    keep = ("whisper-cli.exe", "whisper.dll", "ggml.dll", "ggml-base.dll")
    for name in sorted(os.listdir(os.path.join(SRC, "asr", "bin", "Release"))):
        if not (name in keep or name.startswith("ggml-cpu-")):
            continue
        moves.append((os.path.join(SRC, "asr", "bin", "Release", name),
                      os.path.join(MIRROR_OFF, "asr", "bin", "Release", name)))

    todo, skip, miss, conf = [], 0, [], []
    for s, d in moves:
        st, info = job(s, d)
        if st == "要搬":
            todo.append((s, d))
        elif st == "已一致":
            skip += 1
        elif st == "冲突":
            conf.append(info)
        else:
            miss.append(info)
    print("待搬 %d，已一致 %d，找不到源 %d，目标更新（不搬）%d" % (len(todo), skip, len(miss), len(conf)))
    for m in miss:
        print("  缺源：%s" % m)
    for c in conf:
        print("  冲突：%s 比它的源新 —— 先在 C:\\microros-agent 里核对/拷回，再同步，**这次没有覆盖它**" % c)
    strays = extra_files(moves)
    for p in strays:
        print("  多出来的（不在交付清单里）：%s %.1f MB" % (
            p, os.path.getsize(p) / 1048576.0))
    if check_only:
        for s, d in todo:
            print("  要搬：%s -> %s" % (s, d))
        return 1 if conf else 0
    bad = []
    for s, d in todo:
        os.makedirs(os.path.dirname(d), exist_ok=True)
        shutil.copy2(s, d)
        if md5(s) != md5(d):
            bad.append(d)
    print("搬完 %d 个，校验不符 %d 个" % (len(todo), len(bad)))
    for b in bad:
        print("  不符：%s" % b)
    if strays and "--prune" in sys.argv:
        for p in strays:
            os.remove(p)
        for root, dirs, files in os.walk(os.path.join(MIRROR_OFF, "asr"), topdown=False):
            if not os.listdir(root):
                os.rmdir(root)
        print("清掉 %d 个多出来的文件" % len(strays))
    return 1 if (bad or conf) else 0


if __name__ == "__main__":
    sys.exit(main("--check" in sys.argv))
