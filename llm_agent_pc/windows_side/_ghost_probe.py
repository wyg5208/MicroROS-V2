# -*- coding: utf-8 -*-
"""ghost_room 的重影角度扫一遍：否决门在这类图上还剩多少裁决权。

同一条 0.8 m 真搬动，图上假墙越厚，"候选位姿把穿墙率降下来"这件事就越不可能成立：
- 轻微重影：图和雷达还能自洽（t1 小），否决门说"不是搬动"是有据的；
- 严重重影：连匹配器首选的位置都穿掉大半回波，此时"不是搬动"这句它没资格说。
这一条决定了 THROUGH_JUNK 放哪儿，也决定 _motion_test.py 要补的用例长什么样。
"""
import math

import nav
from _motion_test import ghost_room
from _reloc_test import sweep

CARRY = (0.8, -0.5)

print("%-6s %-7s %-8s %-18s %-16s %-8s %s"
      % ("角度", "墙格", "首选穿", "声称→首选", "差(>0.15 才不赦)", "判不了", "搬动吻合→峰值"))
for ang in (3, 5, 8, 12, 16, 20, 30, 40, 60):
    w = ghost_room(ang)
    pose = (1.0, 0.0, 0.0)
    car = sweep(pose[0] + CARRY[0], pose[1] + CARRY[1], 0.0)
    r = nav.scan_offset(w, pose, car)
    if r is None:
        continue
    t0 = nav.through_wall(w, pose, car)
    t1 = nav.through_wall(w, (pose[0] + r["dx"], pose[1] + r["dy"], 0.0), car)
    clean = nav.through_wall(w, pose, sweep(pose[0], pose[1], 0.0))
    veto = t1 > t0 - 0.15
    print("%-6.0f %-7d %-9.3f %-18s %-9s %-8s 搬动吻合 %.2f 峰值 %.2f"
          % (ang, int((w.lo >= nav.MATCH_WALL).sum()), clean,
             "%.3f -> %.3f" % (t0, t1), "%.3f" % (t0 - t1),
             "赦" if veto and t1 <= 0.30 else ("判不了" if veto else "不停"),
             r["zero"], r["best"]))
