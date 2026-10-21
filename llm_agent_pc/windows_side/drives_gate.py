# -*- coding: utf-8 -*-
"""一道门：会真让轮子转的脚本，必须显式指名才执行。

为什么不是"看 docstring 就行"：2026-10-21 有人把 `_*_test.py` 当离线判据批量跑了一遍，
`_estop_move_test.py`（无参数=前进 0.5 m 再按急停）、`_abort_test.py`、`_machine_stop_test.py`
三个真车脚本就这么执行了 —— 车被开走约 1.9 m，注入的假位姿有 1.07 m 没抵消掉，
那张刚建了十几分钟的图被同一面墙重画两层，只能清空重建。名字像测试、docstring 写着
"真车验证"，都没能拦住，所以拦要落在**执行**上：不给这个词就一行命令都不发。

跑法（原有参数一律往后挪一位，其余不变）：
    python -X utf8 _estop_move_test.py drives stop 0.5
    python -X utf8 _machine_stop_test.py drives 0.8 2
判据：`_drives_gate_test.py`（全离线，不动车）。
"""
import os
import sys

DRIVES_TOKEN = "drives"


def require_drives(what):
    """argv[1] 必须是 `drives`，否则退出码 2 且一个字都不往外发。

    通过时把这个词从 argv 里摘掉：调用方的 `sys.argv[1]`、`[2]`… 因此和加门之前
    指向同一个参数，七个脚本各自的解析一行都不用改。
    """
    argv = sys.argv
    if len(argv) > 1 and argv[1] == DRIVES_TOKEN:
        del sys.argv[1]
        return
    prog = os.path.basename(argv[0])
    print("%s 会真的让轮子转：%s" % (prog, what))
    print("要执行就显式指名：python -X utf8 %s %s [原有参数]" % (prog, DRIVES_TOKEN))
    print("（这一天踩过：批量跑 `_*_test.py` 时三个真车脚本被执行，车开走 1.9 m、图重画两层。）")
    raise SystemExit(2)
