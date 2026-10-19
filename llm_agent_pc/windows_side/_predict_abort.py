# -*- coding: utf-8 -*-
"""Would _motion_watch actually pull the motors if the pose were lying by N metres sideways?

Reads the live sweep, the live map and the live pose, then asks the moving detector's exact gates
at a *hypothetical* corrupted pose -- no motors, no bias written. Run it before the on-car test,
because the direction the offset goes in decides everything: sideways into mapped ground is a
"moved" verdict, sideways into never-measured ground is only "lost", and "lost" needs the
unmeasured share to be small before it aborts.
"""
import json
import math

import numpy as np

import car_web
import nav
from _why_mismatch import Grid, get


def main():
    st, sc = get("/api/state"), get("/api/scan")
    lo = np.load("cache_console/grid.npy")
    ox, oy = json.load(open("cache_console/origin.json"))
    w = Grid(lo, 0.05)
    pose = (st["pose"][0] + ox, st["pose"][1] + oy, math.radians(st["pose"][2]))
    pairs = [(float(a), float(r)) for a, r in sc["pairs"]]
    yaw = pose[2]
    perp = (math.sin(yaw), -math.cos(yaw))
    print("位姿 (%+.2f, %+.2f, %+.1f°)  回波 %d  图 %d 格  电压 %.2f V  busy %r"
          % (pose[0], pose[1], math.degrees(yaw), len(pairs),
             int((lo >= nav.MATCH_WALL).sum()), st["volts"] or -1, st["busy"]))
    rows = [(0.0, 1.0)]                        # the honest pose: a route must be silent here
    rows += [(d, s) for d in (0.6, 1.1, 1.6) for s in (1, -1)]
    for dist, sgn in rows:
        fake = (pose[0] + sgn * dist * perp[0], pose[1] + sgn * dist * perp[1], yaw)
        r = nav.scan_offset(w, fake, pairs, reach=car_web.MOTION_REACH, step=car_web.MOTION_STEP,
                            every=car_web.MOTION_EVERY)
        if r is None:
            print("  %+5.2f m  打分器没答案" % (sgn * dist))
            continue
        t0 = nav.through_wall(w, fake, pairs)
        t1 = nav.through_wall(w, (fake[0] + r["dx"], fake[1] + r["dy"], yaw), pairs)
        # Mirror _motion_watch exactly, gates and all, so this table cannot disagree with the code
        # that would have to pull the motors.
        fits = (r["best"] - r["zero"] >= car_web.WANT_MARGIN
                and math.hypot(r["dx"], r["dy"]) > car_web.DRIFT_MAX)
        unambiguous = (r["best"] >= car_web.RELOC_FLOOR
                       and r["best"] - r["rival"] >= car_web.RELOC_AMBIG)
        check = car_web.Console._veto_check(None, t0, t1)
        moved = False
        if fits:
            if unambiguous:
                moved = check != "veto"
            else:
                moved = check == "ok" and t1 <= t0 - car_web.THROUGH_BETTER
        # The route under consideration: let the *grid* carry the accusation when the score's
        # margin is too thin (a ghosted map dilutes best-zero below WANT_MARGIN). It has to be
        # silent at dist 0, or it is a new false positive, not a new sense.
        accuse = (t0 is not None and t1 is not None and r["best"] >= car_web.RELOC_FLOOR
                  and t1 <= t0 - car_web.THROUGH_BETTER
                  and math.hypot(r["dx"], r["dy"]) > car_web.DRIFT_MAX)
        lost = (r["zero"] < car_web.LOST_FLOOR and r["best"] < car_web.BEST_FLOOR
                and r["outside"] < car_web.MOTION_FRESH)
        if accuse and not fits:
            moved = True
        # The veto is only entitled to excuse while the map can still answer the question. Past
        # THROUGH_JUNK -- and below THROUGH_ACCUSE, where it has no contradiction to point at -- the
        # same "no contrast" verdict becomes a stop, so print which of the four the gate returned.
        call = ("赦免(图能答)" if check == "veto" else
                "判不了->停车" if check == "junk" else
                "没资格赦(声称位姿没被矛盾)->照搬动停" if check == "ok" and not unambiguous else
                "否决门没拦住->照搬动停" if moved else "没答案" if check == "none" else "沉默")
        back = math.hypot(r["dx"] + sgn * dist * perp[0], r["dy"] + sgn * dist * perp[1])
        print("  %+5.2f m  吻合 %.0f%%  峰值 %.0f%% @ (%+.2f,%+.2f)  次优 %.0f%%  未测 %.0f%%"
              "  moved %s lost %s  否决 %s  穿墙 %.0f%%→%.0f%%  峰值离真位姿 %.2f m"
              % (sgn * dist, 100 * r["zero"], 100 * r["best"], r["dx"], r["dy"], 100 * r["rival"],
                 100 * r["outside"], moved, lost, check, 100 * (t0 or -1), 100 * (t1 or -1), back))
        print("      -> %s  （间距 %.3f %s WANT %.2f / 指认 %s）"
              % (call, r["best"] - r["zero"], "过" if fits else "不过", car_web.WANT_MARGIN, accuse))


if __name__ == "__main__":
    main()
