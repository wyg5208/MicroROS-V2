# -*- coding: utf-8 -*-
"""否决门的输入到底有没有信号：在**可信位姿**上量穿墙率，再量几个假想偏移，并扫 lead。

through_wall 想当"地图错了"的裁判，前提是它自己站得住：车此刻位姿被雷达+地图一致认可
（agree≈1）时，这一帧的穿墙率必须很低。若可信位姿上穿墙率本身就有 0.5，那这个数在
这片图上就是饱和的，拿它和候选位姿比出来的差值不能当否决证据 —— 那种情况下要么先修图
（Tour 磨墙那件事），要么换判据，而不是接着调 THROUGH_BETTER。

lead（"越过终点前多少米才算看穿"）就是要扫的那个量。判据是**两头都要**：可信位姿上必须
接近 0（有余量），而假想真搬动处必须明显高于它复原出来的位姿（否则否决门会把真搬动也
赦免掉，正是 2026-10-18 那次 +0.80 m 报成"地图画错了"的形状）。

不写 bias、不动电机：只读活的 state/scan 和盘上的 grid。
"""
import json
import math

import numpy as np

import car_web
import nav
from _why_mismatch import Grid, get

LEADS = (0.10, 0.20, 0.30, 0.45, 0.60)


def main():
    st, sc = get("/api/state"), get("/api/scan")
    lo = np.load("cache_console/grid.npy")
    ox, oy = json.load(open("cache_console/origin.json"))
    w = Grid(lo, 0.05)
    pairs = [(float(a), float(r)) for a, r in sc["pairs"]]
    # car_web 用的是 world（活的）+ pose（含 bias，odom 系），这里用同一套坐标：
    # state.pose 已经减过原点，nav 的世界是 odom 系，所以加回 origin。
    pose = (st["pose"][0] + ox, st["pose"][1] + oy, math.radians(st["pose"][2]))
    yaw = pose[2]
    r = nav.scan_offset(w, pose, pairs)
    print("位姿 (%+.2f, %+.2f, %+.1f°)  回波 %d  占据 %d 格   可信位姿吻合 %s 峰值 %s"
          % (pose[0], pose[1], math.degrees(yaw), len(pairs), int((lo >= 0.65).sum()),
             "%.3f" % r["zero"] if r else "-", "%.3f" % r["best"] if r else None))
    print("判据：可信位姿穿墙率要低（留余量），且搬动处明显高于它复原出的位姿（否则真搬动被赦免）\n")
    print("%-6s %-11s %-11s %-9s %s" % ("lead", "可信位姿", "搬动处峰值", "最小差", "各偏移 声称→峰值"))
    for lead in LEADS:
        t_trust = nav.through_wall(w, pose, pairs, lead=lead)
        cells = []
        gaps = []
        offs = []
        for d in (-0.8, 0.8, 1.6):
            for unit in ((math.sin(yaw), -math.cos(yaw)), (math.cos(yaw), math.sin(yaw))):
                p = (pose[0] + d * unit[0], pose[1] + d * unit[1], yaw)
                rr = nav.scan_offset(w, p, pairs)
                if rr is None:
                    continue
                t0 = nav.through_wall(w, p, pairs, lead=lead)
                t1 = nav.through_wall(w, (p[0] + rr["dx"], p[1] + rr["dy"], yaw), pairs, lead=lead)
                gaps.append(t0 - t1)
                offs.append("%+.1f:%.2f→%.2f%s" % (d, t0, t1, "赦" if t1 > t0 - car_web.THROUGH_BETTER else ""))
        print("%-6.2f %-11s %-9s %s"
              % (lead, "%.3f" % t_trust if t_trust is not None else "-",
                 "%.2f" % (min(gaps) if gaps else 0), " ".join(offs)))


main()
