# -*- coding: utf-8 -*-
"""Which maps make the *score* blind to a real carry, and does the grid still see it?

The live 2026-10-18 failure was `best - zero = 0.219` against `WANT_MARGIN = 0.25`: the gate never
opened, so nothing was ever marked suspicious, the fold loop wrote the lie into the grid, and the
second sample the abort waits for never arrived. The veto work in task 32 cannot help if the code
never reaches the veto.

Here one 0.8 m carry is asked of five maps -- the clean room, a two-fold young one, one smeared by
8° and one by 20° of the chassis' invented yaw, and one that already swallowed an earlier carry --
with both questions printed side by side: the score's (吻合 / 峰值 / 间距) and the grid's
(声称位姿穿墙 t0 -> 首选位姿穿墙 t1). The column that decides the design is 间距 < WANT 而 t0-t1 >=
THROUGH_BETTER: a map that blinds the score but not the grid.
"""
import math

import car_web
import nav
from _motion_test import ghost_room, room, straight, thin_room
from _reloc_test import sweep

CARRY = (0.8, -0.5)


def healed_room(carry=CARRY, x0=-1.0, x1=0.3, step=0.16, every=3, samples=12):
    """The room the way `_fold` leaves it after a carry was *not* held out: every sweep voted at
    the lying pose, so the grid now owns two rooms, one of them fiction. This is the map the car
    was standing on when the live run went blind.

    `x1 - x0` is how far the lie got folded before the map stopped being a clean room: the live
    failure was measured one second into a drive, which is two or three frames, not nine.
    """
    w = nav.World(extent=24.0, res=0.05)
    for p in straight(x0, x1, step=step):
        real = (p[0] + carry[0], p[1] + carry[1])
        w.integrate((p[0], p[1], 0.0), sweep(real[0], real[1], 0.0), every=every, samples=samples)
    return w


def show(tag, w, claims, carry=CARRY):
    print("%s  图 %d 格" % (tag, int((w.lo >= nav.MATCH_WALL).sum())))
    for p in claims:
        # the car claims `p`; the sweep is whatever is really in front of the carried car
        real = (p[0] + carry[0], p[1] + carry[1])
        pairs = sweep(real[0], real[1], 0.0, n=667)
        r = nav.scan_offset(w, (p[0], p[1], 0.0), pairs, reach=car_web.MOTION_REACH,
                            step=car_web.MOTION_STEP, every=car_web.MOTION_EVERY)
        if r is None:
            print("   %+.2f m  打分器没答案" % p[0])
            continue
        t0 = nav.through_wall(w, (p[0], p[1], 0.0), pairs)
        t1 = nav.through_wall(w, (p[0] + r["dx"], p[1] + r["dy"], 0.0), pairs)
        off = math.hypot(r["dx"], r["dy"])
        fits = r["best"] - r["zero"] >= car_web.WANT_MARGIN and off > car_web.DRIFT_MAX
        accuse = (None not in (t0, t1) and t1 <= t0 - car_web.THROUGH_BETTER
                  and off > car_web.DRIFT_MAX)
        print("   %+.2f m  吻合 %-5s 峰值 %-5s 间距 %-6s %s  峰 @(%+.2f,%+.2f) 次优 %-5s 未测 %-5s"
              "  穿墙 %s -> %s  Δ%+0.3f  fits %s 指认 %s"
              % (p[0], fmt(r["zero"]), fmt(r["best"]), fmt(r["best"] - r["zero"]),
                 "过" if fits else "瞎", r["dx"], r["dy"], fmt(r["rival"]), fmt(r["outside"]),
                 fmt(t0), fmt(t1), -1 if None in (t0, t1) else t1 - t0, fits, accuse))


def fmt(v):
    return "  --  " if v is None else "%.3f" % v


def online(carry=CARRY, step=0.16, frames=9, every=3, samples=12, start=None):
    """Replay the live sequence frame by frame: fold the first k lying sweeps into the grid, then
    ask what the detector sees on sweep k+1. That is the only order that matters -- the map it
    judges is the one the previous frames made, and the whole point of task 33 is that it must not
    include this frame's own vote.

    `start` is the map the car was honest about *before* the jump. It is not optional detail: fold
    a constant pose offset into an empty grid and the grid comes out *consistent with the lie*, so
    every metric reads 0.99 and no detector on earth can see anything. The car's live failure was
    a jump into a map that already existed, and that is the state being replayed here.
    """
    w = room() if start is None else start()
    print("在线重放：每帧先把上一帧折进图，再问这一帧（谎 = 声称位姿差 %s）" % (carry,))
    print("   帧  图格数  吻合   峰值   间距  门的  峰 @(dx,dy)      穿墙 t0 -> t1   Δ      fits 指认")
    for i in range(frames):
        claim = (-1.0 + i * step, 0.0)
        real = (claim[0] + carry[0], claim[1] + carry[1])
        pairs = sweep(real[0], real[1], 0.0, n=667)
        r = nav.scan_offset(w, (claim[0], claim[1], 0.0), pairs, reach=car_web.MOTION_REACH,
                            step=car_web.MOTION_STEP, every=car_web.MOTION_EVERY)
        cells = int((w.lo >= nav.MATCH_WALL).sum())
        if r is None:
            print("   %2d  %6d  打分器没答案（图太空）" % (i + 1, cells))
        else:
            t0 = nav.through_wall(w, (claim[0], claim[1], 0.0), pairs)
            t1 = nav.through_wall(w, (claim[0] + r["dx"], claim[1] + r["dy"], 0.0), pairs)
            off = math.hypot(r["dx"], r["dy"])
            fits = r["best"] - r["zero"] >= car_web.WANT_MARGIN and off > car_web.DRIFT_MAX
            accuse = (None not in (t0, t1) and t1 <= t0 - car_web.THROUGH_BETTER
                      and off > car_web.DRIFT_MAX)
            print("   %2d  %6d  %-6s %-6s %-6s %-4s (%+.2f,%+.2f)  %s -> %s  %+0.3f  %s %s"
                  % (i + 1, cells, fmt(r["zero"]), fmt(r["best"]), fmt(r["best"] - r["zero"]),
                     "过" if fits else "瞎", r["dx"], r["dy"], fmt(t0), fmt(t1),
                     -1 if None in (t0, t1) else t1 - t0, fits, accuse))
        w.integrate((claim[0], claim[1], 0.0), pairs, every=every, samples=samples)


def main():
    claims = [(-0.68, 0.0, 0.0), (-0.2, 0.0, 0.0), (0.28, 0.0, 0.0)]
    show("干净房间", room(), claims)
    show("稀疏图（两次折叠）", thin_room(2), claims)
    show("重影 8°", ghost_room(8.0), claims)
    show("重影 20°", ghost_room(20.0), claims)
    print()
    online()
    print()
    online(carry=(0.4, -0.25))


main()
