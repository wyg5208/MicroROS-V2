# -*- coding: utf-8 -*-
"""Offline check for nav.scan_offset: does a synthetic carry show up as the offset it was?

Nothing here needs the car. The room is a rectangle, the sweep is an exact ray-cast against
it, and the map is built by integrating that sweep at the *true* poses -- so the only thing
under test is whether the matcher recovers the discrepancy between the pose we hand it and
the pose the sweep was really taken at.
"""
import json
import math
import os

import numpy as np

import nav

ROOM = (-3.0, 3.0, -2.0, 2.0)


def sweep(px, py, yaw, n=667, rmax=8.0):
    """Body-frame (deg, range) for a full ring cast against the room rectangle."""
    out = []
    for i in range(n):
        a = yaw + 2.0 * math.pi * i / n
        ca, sa = math.cos(a), math.sin(a)
        t = rmax
        for edge, axis in ((ROOM[0], "x"), (ROOM[1], "x"), (ROOM[2], "y"), (ROOM[3], "y")):
            c, d = (px, ca) if axis == "x" else (py, sa)
            if abs(d) < 1e-9:
                continue
            s = (edge - c) / d
            if s > 1e-6:
                t = min(t, s)
        if t < rmax:
            out.append(((360.0 * i / n + 180.0) % 360.0 - 180.0, t))
    return out


def build():
    w = nav.World(extent=24.0, res=0.05)
    for py in (-1.0, 0.0, 1.0):
        for px in (-1.5, 0.0, 1.5):
            w.integrate((px, py, 0.0), sweep(px, py, 0.0), every=3, samples=12)
    return w


def main():
    w = build()
    print("map walls: %d cells (>= %.2f)" % (int((w.grid() >= nav.MATCH_WALL).sum()), nav.MATCH_WALL))

    true = (1.0, 0.5, 0.3)
    carried = (0.8, -0.5)

    r = nav.scan_offset(w, true, sweep(*true))
    print("honest pose   -> zero %.3f best %.3f at (%+.2f, %+.2f) support %d"
          % (r["zero"], r["best"], r["dx"], r["dy"], r["support"]))
    assert r["zero"] > 0.9, "a correct pose must match its own map"
    assert abs(r["dx"]) < 0.11 and abs(r["dy"]) < 0.11, "no carry should be invented"

    r2 = nav.scan_offset(w, true, sweep(true[0] + carried[0], true[1] + carried[1], true[2]))
    print("after  carry  -> zero %.3f best %.3f at (%+.2f, %+.2f) support %d"
          % (r2["zero"], r2["best"], r2["dx"], r2["dy"], r2["support"]))
    err = math.hypot(r2["dx"] - carried[0], r2["dy"] - carried[1])
    assert r2["zero"] < 0.5, "a real carry must collapse the zero-offset score"
    assert r2["best"] - r2["zero"] > 0.3, "and the search must find it"
    assert err < 0.15, "recovered offset off by %.2f m" % err

    turned = true[2] + math.radians(25.0)
    r3 = nav.scan_offset(w, (true[0], true[1], turned),
                         sweep(true[0] + carried[0], true[1] + carried[1], turned))
    print("carried+turned-> zero %.3f best %.3f at (%+.2f, %+.2f)"
          % (r3["zero"], r3["best"], r3["dx"], r3["dy"]))
    assert math.hypot(r3["dx"] - carried[0], r3["dy"] - carried[1]) < 0.15, \
        "a carry the gyro already reported in yaw must still be found by translation alone"

    # Sub-cell accuracy, measured against offsets that deliberately miss the 5 cm grid. Without
    # this the drift corrector cannot tell 3 cm of real slip from the staircase it lives on.
    # The sweep is taken at the *displaced* place while the pose claims the honest one, which is
    # the same convention as the carry case above: the answer is the offset to add to the pose.
    print("\nsub-cell refinement (sweep displaced by a known, non-grid amount):")
    errs_q, errs_f = [], []
    for ex, ey in ((0.03, -0.02), (-0.07, 0.04), (0.11, -0.09), (-0.02, 0.13), (0.08, 0.08)):
        rr = nav.scan_offset(w, true, sweep(true[0] + ex, true[1] + ey, true[2]))
        eq = math.hypot(rr["dx"] - ex, rr["dy"] - ey)
        ef = math.hypot(rr["fdx"] - ex, rr["fdy"] - ey)
        errs_q.append(eq); errs_f.append(ef)
        print("  true (%+.3f,%+.3f)  grid (%+.2f,%+.2f) err %.3f | refined (%+.3f,%+.3f) err %.3f"
              % (ex, ey, rr["dx"], rr["dy"], eq, rr["fdx"], rr["fdy"], ef))
    print("  mean err: grid %.3f m -> refined %.3f m" % (np.mean(errs_q), np.mean(errs_f)))
    assert np.mean(errs_f) < np.mean(errs_q), "refinement must beat the 5 cm staircase"

    t0 = __import__("time").monotonic()
    for _ in range(10):
        nav.scan_offset(w, true, sweep(*true))
    print("10 searches in %.0f ms" % ((__import__("time").monotonic() - t0) * 100))

    # How much room the matcher has to *refuse*: in a rectangle the walls are parallel, so a
    # rival placement is expected, and the size of its score is what decides whether moving the
    # pose is a correction or a guess. Printed per case so the thresholds are chosen on numbers.
    for name, rr in (("honest", r), ("carried", r2), ("turned", r3)):
        print("%-7s best %.3f rival %.3f at (%+.2f, %+.2f)  gap %.3f  -> %s"
              % (name, rr["best"], rr["rival"], rr["rdx"], rr["rdy"],
                 rr["best"] - rr["rival"],
                 "auto" if (rr["best"] >= 0.85 and rr["best"] - rr["rival"] >= 0.12)
                 else "ask a human"))

    thin = nav.World()
    thin.integrate((0, 0, 0), sweep(0, 0, 0, n=40), every=30)
    print("thin map -> %s (must be None: nothing to match against)"
          % nav.scan_offset(thin, (0, 0, 0), sweep(*true)))

    # ---- the veto: does the winning placement *explain* the sweep, or only score better? ----
    # scan_offset answers one question (does each return land on a wall) and a wrongly drawn map
    # answers it badly everywhere, so its argmax slides onto the junk and reports a carry. This is
    # the false alarm measured on the real car on 2026-10-18, reproduced from a saved fixture, and
    # the two synthetic controls that say the veto is not simply a veto.
    print("\nthrough-wall veto:")
    t_honest = nav.through_wall(w, true, sweep(*true))
    print("  honest pose, clean map     -> %.3f contradicted (expect ~0)" % t_honest)
    assert t_honest < 0.05, "a correct pose on a correct map must not see through walls"

    r2b = nav.scan_offset(w, true, sweep(true[0] + carried[0], true[1] + carried[1], true[2]))
    t0 = nav.through_wall(w, true, sweep(true[0] + carried[0], true[1] + carried[1], true[2]))
    t1 = nav.through_wall(w, (true[0] + r2b["dx"], true[1] + r2b["dy"], true[2]),
                          sweep(true[0] + carried[0], true[1] + carried[1], true[2]))
    print("  real carry                 -> claimed %.3f, recovered %.3f (veto must NOT fire)"
          % (t0, t1))
    assert t0 > 0.2 and t1 < t0 - 0.15, "a genuine carry must survive the veto"

    # A phantom box around the car: the sweep is honest, the pose is honest, and the score cannot
    # tell (the real far walls still sit behind the junk, so every return still lands on a wall).
    # But every ray now passes through a surface that is not there. This is what the real map fell
    # into after the tour ground itself into furniture, and why the alarm said "somebody moved
    # the car": the score dropped, the argmax slid onto the junk, and nothing asked the map.
    pw = nav.World(extent=24.0, res=0.05)
    pw.lo = w.lo.copy()
    pw.frames = w.frames
    R = 1.5
    for s in np.arange(-R, R + 1e-9, 0.05):
        for (dx, dy) in ((R, s), (-R, s), (s, R), (s, -R)):
            pw._add(np.array([true[0] + dx]), np.array([true[1] + dy]), pw.v_occ)
    sw = sweep(*true)
    r_ph = nav.scan_offset(pw, true, sw)
    t_ph = nav.through_wall(pw, true, sw)
    print("  honest pose, phantom box   -> score %.3f (looks fine) but %.3f sees through it"
          % (r_ph["zero"], t_ph))
    assert r_ph["zero"] > 0.9, "the case must be invisible to the score, or it tests nothing"
    assert t_ph > 0.3, "through_wall must catch junk the score is blind to"

    # The real fixture: the matcher wanted (-1.5, -0.4) on a map the sweep was seeing through.
    fx = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "_fixture_case.json"), encoding="utf-8"))
    fg = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "_fixture_grid.npy"))
    fw = nav.World(extent=24.0, res=0.05)
    fw.lo = fg
    fp = (fx["pose"][0], fx["pose"][1], math.radians(fx["pose"][2]))
    fr = nav.scan_offset(fw, fp, fx["pairs"])
    f0 = nav.through_wall(fw, fp, fx["pairs"])
    f1 = nav.through_wall(fw, (fp[0] + fr["dx"], fp[1] + fr["dy"], fp[2]), fx["pairs"])
    print("  2026-10-18 false alarm     -> matcher wants (%+.2f,%+.2f) at %.0f%% vs %.0f%%, "
          "contradicted %.3f -> %.3f"
          % (fr["dx"], fr["dy"], 100 * fr["best"], 100 * fr["zero"], f0, f1))
    # The margin that tripped the alarm was 0.306 (log: 52% claimed, 83% at (-1.60,-0.50)); the
    # room has moved since this sweep was saved and the margin is now smaller, so the margin is
    # printed rather than asserted. What is stable, and what the veto actually turns on, is that
    # the placement the matcher wanted explains the sweep *worse* than where the odometry stands.
    print("     (score margin %.3f; WANT_MARGIN 0.25 at alarm time)" % (fr["best"] - fr["zero"]))
    assert f0 > 0.2 and f1 > f0 + 0.1, \
        "on the saved map every candidate must contradict the sweep more than the claimed pose"
    print("OK")


if __name__ == "__main__":
    main()
