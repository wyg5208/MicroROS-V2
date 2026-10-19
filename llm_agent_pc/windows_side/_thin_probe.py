# -*- coding: utf-8 -*-
"""Where does a *young* map sit on the two see-through shares, and does a real carry still stop?

_task 32's acceptance case, run before it is written into _motion_test.py_: 稀疏图 + 真搬动 = 必须停.
The veto gate argues "this map contradicts the sweep, so a better score somewhere else proves
nothing". That argument needs a contradiction to point at. A map built from two or three folds has
almost no claimed walls, so both shares come out near zero, "the candidate improved nothing" is
trivially true -- and the veto would excuse a real carry on the strength of an argument the thin map
never made. Print zero/best/rival/support and the t0 -> t1 pair for the same 0.8 m carry the suite
uses, across map ages, so the floor comes from the numbers rather than from a guess.

Run: C:\\microros-agent\\pc_side\\_rosrun.bat _thin_probe.py
"""
import math

import car_web
import nav
from _reloc_test import sweep


def room_cells():
    return [(px, py) for py in (-1.0, 0.0, 1.0) for px in (-1.5, 0.0, 1.5)]


def build(keep):
    w = nav.World(extent=24.0, res=0.05)
    for (px, py) in room_cells()[:keep]:
        w.integrate((px, py, 0.0), sweep(px, py, 0.0), every=3, samples=12)
    return w


def main():
    print("%-6s %-7s %-7s %-7s %-7s %-7s %-8s %s"
          % ("折几次", "墙格", "吻合", "峰值", "次优", "未测", "支持", "声称->首选  尖锐 moved lost 门"))
    for keep in (1, 2, 3, 4, 6, 9):
        w = build(keep)
        p = (0.0, 0.0, 0.0)
        pairs = sweep(0.8, -0.5, 0.0, n=667)          # the car stands where odometry says, sees 0.8 m away
        r = nav.scan_offset(w, p, pairs, reach=car_web.MOTION_REACH, step=car_web.MOTION_STEP,
                            every=car_web.MOTION_EVERY)
        if r is None:
            print("%-6d %-7d  打分器没答案（墙格 < nav.MIN_WALLS=%d）：这条本来就没资格判断"
                  % (keep, int((w.lo >= nav.MATCH_WALL).sum()), nav.MIN_WALLS))
            continue
        t0 = nav.through_wall(w, p, pairs)
        t1 = nav.through_wall(w, (p[0] + r["dx"], p[1] + r["dy"], p[2]), pairs)
        fits = (r["best"] - r["zero"] >= car_web.WANT_MARGIN
                and math.hypot(r["dx"], r["dy"]) > car_web.DRIFT_MAX)
        unambiguous = (r["best"] >= car_web.RELOC_FLOOR
                       and r["best"] - r["rival"] >= car_web.RELOC_AMBIG)
        check = car_web.Console._veto_check(None, t0, t1)
        moved = False
        if fits:
            moved = (check != "veto" if unambiguous
                     else check == "ok" and t1 <= t0 - car_web.THROUGH_BETTER)
        lost = (r["zero"] < car_web.LOST_FLOOR and r["best"] < car_web.BEST_FLOOR
                and r["outside"] < car_web.MOTION_FRESH)
        print("%-6d %-7d %-7.3f %-7.3f %-7.3f %-7.3f %-8d %.3f->%.3f  尖锐 %s moved %s lost %s 门 %s"
              % (keep, r["walls"], r["zero"], r["best"], r["rival"], r["outside"], r["support"],
                 t0, t1, unambiguous, moved, lost, check))


if __name__ == "__main__":
    main()
