# -*- coding: utf-8 -*-
"""_sync_mirror.job() 的守门测试：**目标比源新时绝不覆盖**。

存在的理由是一次真事故：2026-10-19 深夜在 G: 那一侧改了《用户操作手册.md》（73 行），
没先拷回权威目录 C:\\microros-agent，随后 `_sync_mirror.py` 把文档也当作它镜像的东西之一
（DOCS 那一组），用 C: 的旧副本盖了回去 —— 而且 `git status` 里连痕迹都不留，因为文件"同步成功"
变回了 HEAD 的样子。代码侧同一件事会咬人（agent.py 那次靠手工拷贝救回），所以规则要变成工具里的门：

    python _sync_conflict_test.py      :: 全离线，不碰车、不碰真镜像

跑过 4 条：目标更新=冲突、目标更旧=要搬、字节相同=已一致、没有源=缺源。
另外钉住"两边 mtime 相同但内容不同 = 要搬"：copy2 会把这个脚本自己搬过去的文件 mtime 对齐，
那种情况必须照搬，否则同步完第二次 check 会说"目标更新"，工具就自己把自己锁死了。
"""
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _sync_mirror as m

d = tempfile.mkdtemp(prefix="_sync_probe_")
src = os.path.join(d, "a.txt")
dst = os.path.join(d, "b.txt")
now = time.time()


def write(p, s):
    with open(p, "w") as fh:
        fh.write(s)


def case(label, want, s_age, d_age, s_txt, d_txt):
    write(src, s_txt)
    os.utime(src, (now - s_age, now - s_age))
    write(dst, d_txt)
    os.utime(dst, (now - d_age, now - d_age))
    got = m.job(src, dst)[0]
    print("%-26s -> %s" % (label, got))
    assert got == want, "%s 应该是 %s，实得 %s" % (label, want, got)


case("目标比源新（有人在镜像侧撰稿）", "冲突", 100, 0, "权威\n", "镜像上写的\n")
case("目标比源旧（源领先）", "要搬", 0, 200, "权威\n", "过期\n")
case("字节相同（刚同步过）", "已一致", 0, 500, "权威\n", "权威\n")
case("mtime 相同但内容不同", "要搬", 0, 0, "权威\n", "过期\n")
got = m.job(os.path.join(d, "missing.txt"), dst)[0]
print("%-26s -> %s" % ("没有源", got))
assert got == "缺源"
print("OK  5/5")
