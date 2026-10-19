# -*- coding: utf-8 -*-
"""Offline check for the drift suppressor: does it correct what is real and refuse what is noise?

Two things are tested, both without the car.

(1) The controller's arithmetic, replayed against the 21 moving samples the real car actually
    produced on 2026-10-18 (/api/drift). Those rows are the reason DRIFT_BEST/ZERO/MAX exist: the
    12 sweeps that still matched the grid agree with each other to 8 mm, the 9 that did not fly to
    +-0.6 m and pin on the edge of the search window. If the gates work, the noisy nine must never
    touch the pose, and whatever the good twelve pay out must stay inside the error budget that the
    distance driven can physically account for.

(2) Closed loop, on a synthetic room whose map was folded at the *true* poses while odometry
    accumulated a forward scale error. Here the matcher genuinely has something to find, and the
    question is whether spending the residual as it arrives keeps the pose closer to the truth and
    the walls single-cell, compared with folding the same drive with the corrector switched off.

Run: python -X utf8 _drift_test.py
"""
import math
import threading

import car_web
import nav
from _edge_test import bands
from _reloc_test import sweep


class FakeCar:
    """Only the bias bookkeeping _drift_correct uses."""

    def __init__(self):
        self.lock = threading.Lock()
        self.bias = (0.0, 0.0)

    def nudge_bias(self, dx, dy):
        with self.lock:
            self.bias = (self.bias[0] + dx, self.bias[1] + dy)
        return self.bias

    def set_bias(self, bx, by):
        with self.lock:
            self.bias = (bx, by)


class Stub:
    """The attributes _drift_correct reads and writes, and nothing else.

    Deliberately not a real Console: this has to run without a bridge, a lidar or a car, and the
    logic under test is a controller, not the plumbing around it.
    """

    def __init__(self, bias0=(0.0, 0.0)):
        self.lock = threading.Lock()
        self.car = FakeCar()
        # The real car is never handed a zero bias: it boots onto a grid saved under some other
        # odom origin, so the restored (+4.43, +3.56) here is the normal case, not an edge one.
        self.car.set_bias(bias0[0], bias0[1])
        self.drift = {"res": None, "agree": None, "why": None, "last": None,
                      "est": [0.0, 0.0], "roll": 0.0, "total": 0.0, "n": 0,
                      "bias": [bias0[0], bias0[1]]}
        self.drift_est = [0.0, 0.0]
        self.drift_est_at = 0.0
        self.drift_roll = 0.0
        self.drift_total = 0.0
        self.drift_n = 0
        self.drift_bias0 = None
        self.logs = []
        self.frozen = 0
        self.because = None

    def push(self, text, kind="info"):
        self.logs.append((kind, text))

    def _save_bias(self):
        pass

    def _freeze_carry(self, r, moved, quiet=False, ask=False, because=None):
        self.frozen += 1
        self.because = because


def _r(fdx, fdy, zero, best):
    return {"fdx": fdx, "fdy": fdy, "dx": round(fdx / 0.05) * 0.05, "dy": round(fdy / 0.05) * 0.05,
            "zero": zero, "best": best, "rival": 0.0, "rdx": 0.0, "rdy": 0.0, "support": 150}


def feed(rows):
    """Replay /api/drift rows through the controller, on their real timestamps."""
    st = Stub()
    for (t, fdx, fdy, zero, best, _sup, moved, _yaw) in rows:
        car_web.Console._drift_correct(st, _r(fdx, fdy, zero, best), moved, t)
    return st


# (t, fdx, fdy, zero, best, support, moved, yaw) -- real car, tour, 2026-10-18, /api/drift.
# The first twelve are a car matching its own map to a couple of centimetres; the last nine are the
# same car with the match gone, hunting a peak across the whole 0.6 m window.
REAL = [
    (22.4, -0.017, -0.005, 0.988, 0.988, 161, 0.03, -83.2),
    (23.5, -0.024, -0.000, 0.973, 0.973, 161, 0.03, -83.2),
    (25.2, -0.024, -0.004, 0.974, 0.974, 156, 0.04, -82.5),
    (25.7, -0.011, +0.002, 0.979, 0.979, 160, 0.03, -84.4),
    (27.6, -0.016, -0.009, 0.969, 0.969, 156, 0.04, -83.6),
    (27.8, -0.009, +0.003, 0.964, 0.964, 156, 0.03, -84.4),
    (28.3, -0.002, -0.012, 0.977, 0.977, 153, 0.03, -83.4),
    (29.6, -0.002, -0.019, 0.940, 0.940, 158, 0.03, -89.6),
    (41.8, -0.084, -0.042, 0.818, 0.854, 161, 0.05, -56.0),
    (42.1, -0.087, -0.039, 0.837, 0.901, 158, 0.04, -50.2),
    (42.7, -0.066, -0.050, 0.836, 0.888, 159, 0.03, -39.8),
    (42.9, -0.071, -0.038, 0.835, 0.857, 162, 0.03, -37.4),
    (43.3, -0.148, +0.013, 0.758, 0.787, 153, 0.03, -44.6),
    (45.1, +0.457, -0.367, 0.459, 0.628, 134, 0.06, 10.9),
    (54.4, -0.285, +0.231, 0.612, 0.700, 152, 0.05, -89.4),
    (56.2, -0.219, +0.301, 0.593, 0.697, 152, 0.04, -89.0),
    (56.7, -0.104, +0.550, 0.573, 0.714, 151, 0.06, -86.7),
    (57.2, -0.116, +0.600, 0.565, 0.720, 149, 0.05, -86.0),
    (57.8, -0.143, +0.600, 0.572, 0.737, 147, 0.07, -82.9),
    (60.0, -0.091, +0.600, 0.441, 0.635, 153, 0.03, -126.2),
    (60.9, -0.409, +0.600, 0.397, 0.605, 142, 0.05, -136.2),
]
NOISY = REAL[12:]


def build_map():
    w = nav.World(extent=24.0, res=0.05)
    for py in (-1.0, 0.0, 1.0):
        for px in (-1.5, 0.0, 1.5):
            w.integrate((px, py, 0.0), sweep(px, py, 0.0), every=3, samples=12)
    return w


def sharpness(w):
    """Mean Chebyshev distance from an occupied cell to open space: 1.0 is a single-cell wall."""
    m = (w.grid() >= nav.MATCH_WALL).copy()
    m[:, :1] = m[:, -1:] = m[:1, :] = m[-1:, :] = False     # np.roll wraps, so clamp the frame
    d = bands(m)
    return (float(d[m].mean()) if m.any() else 0.0), int(m.sum())


def rectangle(step=0.12):
    path = []
    for x0, y0, x1, y1 in ((-1.5, -1.0, 1.5, -1.0), (1.5, -1.0, 1.5, 1.0),
                           (1.5, 1.0, -1.5, 1.0), (-1.5, 1.0, -1.5, -1.0)):
        n = int(round(math.hypot(x1 - x0, y1 - y0) / step))
        for i in range(n):
            path.append((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n,
                         math.atan2(y1 - y0, x1 - x0)))
    return path


def closed_loop(rate, correct):
    """Drive a rectangle with odometry gaining `rate` metres of error per metre travelled.

    The map is folded at the pose odometry claims, which is what the car really does, so with the
    suppressor off the error shows up only as smeared walls; with it on, every sweep that still
    explains itself feeds a centimetre back into the pose. `ex, ey` is the error *left* after
    whatever correction has already been paid out, and paying out the controller's step is what
    shrinks it.
    """
    w = build_map()
    st = Stub()
    ex = ey = 0.0
    worst = 0.0
    path = rectangle()
    for t, (tx, ty, yaw) in enumerate(path):
        rx, ry = tx - ex, ty - ey
        worst = max(worst, math.hypot(ex, ey))
        sw = sweep(tx, ty, yaw)
        if correct:
            before = st.car.bias
            r = nav.scan_offset(w, (rx, ry, yaw), sw, reach=car_web.DRIFT_REACH,
                                step=car_web.DRIFT_STEP, every=car_web.DRIFT_EVERY)
            if r is not None:
                car_web.Console._drift_correct(st, r, 0.12, t * 0.2)
                ex -= st.car.bias[0] - before[0]
                ey -= st.car.bias[1] - before[1]
        ex += rate * 0.12 * math.cos(yaw)
        ey += rate * 0.12 * math.sin(yaw)
        w.integrate((rx, ry, yaw), sw, every=3, samples=12)
    th, occ = sharpness(w)
    return {"final": math.hypot(ex, ey), "worst": worst, "thick": th, "occ": occ,
            "applied": st.drift_n, "total": st.drift_total, "frozen": st.frozen,
            "len": len(path) * 0.12}


def main():
    print("gates: best>=%.2f zero>=%.2f |res|<=%.2f  dead %.2f m gain %.2f cap %.2f m "
          "rate %.2f m/m budget %.2f m"
          % (car_web.DRIFT_BEST, car_web.DRIFT_ZERO, car_web.DRIFT_MAX, car_web.DRIFT_DEAD,
             car_web.DRIFT_GAIN, car_web.DRIFT_CAP, car_web.DRIFT_RATE, car_web.DRIFT_TOTAL))

    # ---- (1a) pure noise must not move the car at all
    st = feed(NOISY)
    print("\nnoise-only replay (%d rows, %.2f m driven): applied %d nudges, bias (%+.3f, %+.3f)"
          % (len(NOISY), sum(r[6] for r in NOISY), st.drift_n, st.car.bias[0], st.car.bias[1]))
    assert st.drift_n == 0 and st.car.bias == (0.0, 0.0), "a flat search surface must be refused"
    assert st.frozen == 0

    # ---- (1b) the real mixed log: bounded, in the right direction, never a freeze
    st = feed(REAL)
    driven = sum(r[6] for r in REAL)
    paid = math.hypot(*st.car.bias)
    print("full replay    (%d rows, %.2f m driven): applied %d, paid out %.4f m "
          "(budget %.4f m), estimate now %s, bias (%+.3f, %+.3f)"
          % (len(REAL), driven, st.drift_n, paid, car_web.DRIFT_RATE * driven,
             st.drift_est, st.car.bias[0], st.car.bias[1]))
    assert paid <= car_web.DRIFT_RATE * driven + 1e-9, \
        "paid %.3f m of correction for %.2f m driven -- more than odometry could have erred" % (
            paid, driven)
    assert st.drift_n < len(REAL), "every sample paying out is not suppression, it is a relay"
    assert st.frozen == 0, "0.87 m driven cannot exhaust the budget"
    assert st.car.bias[0] <= 0.0, "the map asked for -x, the pose must go toward -x"
    assert st.drift_total < car_web.DRIFT_TOTAL

    # ---- (1c) a bias the matcher keeps insisting on is paid at most at the physical rate
    st = Stub()
    for i in range(200):
        car_web.Console._drift_correct(st, _r(-0.06, 0.04, 0.95, 0.95), 0.04, i * 0.2 + 0.1)
    print("constant 7 cm error, %.1f m driven: applied %d, bias (%+.3f, %+.3f), paid %.3f m"
          % (200 * 0.04, st.drift_n, st.car.bias[0], st.car.bias[1], math.hypot(*st.car.bias)))
    assert math.hypot(*st.car.bias) <= car_web.DRIFT_RATE * 200 * 0.04 + 1e-9, "runaway"
    assert st.drift_n > 3, "a steady error must be acted on, not merely estimated forever"
    assert st.frozen == 0, "%.2f m of bias is not the end of the world" % (math.hypot(*st.car.bias))

    # ---- (1c') a wobble that averages out must never be paid out at all: this is what the
    # average is for. The summed version of this controller spent 0.11 m of correction on exactly
    # this signal, at the rate cap, for the whole tour.
    st = Stub()
    for i in range(400):
        alt = 0.09 if i % 2 else -0.09
        car_web.Console._drift_correct(st, _r(alt, 0.0, 0.95, 0.95), 0.04, i * 0.2 + 0.1)
    print("+-9 cm alternating wobble, %.0f m driven: applied %d, bias (%+.3f, %+.3f)"
          % (400 * 0.04, st.drift_n, st.car.bias[0], st.car.bias[1]))
    assert st.drift_n <= 2 and math.hypot(*st.car.bias) < 0.05,         "noise averaged out but the pose still moved %.3f m" % math.hypot(*st.car.bias)

    # ---- (1d) ... but a residual that never settles has to hand over, not pace on forever
    # In the fold loop a handover is terminal: _freeze_carry drops `mapping`, and _drift_probe
    # refuses to work while the reloc banner is frozen. The stub does neither, so stop the loop the
    # same way the car would rather than let it bank up another dozen handovers.
    st = Stub()
    crossed = None
    for i in range(900):
        if st.frozen:
            break
        car_web.Console._drift_correct(st, _r(-0.06, 0.0, 0.95, 0.95), 0.04, i * 0.2 + 0.1)
        if crossed is None and st.frozen:
            crossed = math.hypot(*st.car.bias)
    print("same residual over %.0f m driven: applied %d, bias (%+.2f, %+.2f), handed over at "
          "%.2f m after %d sample(s)"
          % (i * 0.04, st.drift_n, st.car.bias[0], st.car.bias[1], crossed, i))
    assert crossed is not None, "a pose walked %.2f m by the corrector must stop and ask" % (
        math.hypot(*st.car.bias))
    assert car_web.DRIFT_TOTAL <= crossed <= car_web.DRIFT_TOTAL + car_web.DRIFT_CAP, \
        "it handed over at %.2f m, not at the %.2f m budget" % (crossed, car_web.DRIFT_TOTAL)

    # ---- (1d') the same run, but handed a bias that is already 5.7 m off zero.
    # A session always boots onto a grid saved under some other odom origin (this car: +4.43/+3.56),
    # so "how wrong is the pose" and "how far has the corrector walked it" are different numbers.
    # The first version compared the *absolute* bias to the budget, which declared bankruptcy on the
    # very first payout and e-stopped the Move that was running at the time.
    st = Stub(bias0=(4.43, 3.56))
    first = None
    crossed = None
    for i in range(900):
        car_web.Console._drift_correct(st, _r(-0.06, 0.0, 0.95, 0.95), 0.04, i * 0.2 + 0.1)
        if first is None and st.drift_n:
            first = (math.hypot(st.car.bias[0] - 4.43, st.car.bias[1] - 3.56),
                     math.hypot(*st.car.bias))
        if st.frozen:
            crossed = math.hypot(st.car.bias[0] - 4.43, st.car.bias[1] - 3.56)
            break
    print("restored (+4.43,+3.56): 第一次修正只走了 %.3f m（绝对 bias %.2f m），"
          "交回人工时自己走了 %.2f m，放弃时写的判词=%s"
          % (first[0], first[1], crossed, "有" if st.because else "没有"))
    assert first[0] < car_web.DRIFT_CAP, "第一笔修正就该很小，却被拿去和整段预算比"
    assert first[1] > car_web.DRIFT_TOTAL, "绝对 bias 没超过预算的话，这个用例就没测到参照系"
    assert crossed is not None and \
        car_web.DRIFT_TOTAL <= crossed <= car_web.DRIFT_TOTAL + car_web.DRIFT_CAP, \
        "预算该在抑制器自己走满 %.2f m 时到期，实际 %.2f m" % (car_web.DRIFT_TOTAL, crossed)
    assert st.because, "机器自己按的停必须带自己的判词，不能借\"疑似被搬动\"那句"

    # ---- (1e) the same residual, but the wheels have not turned: nothing may be spent.
    # moved=0 is what the fold loop reports when odometry itself is frozen (a stalled bridge or
    # a car held up off the floor), so the rate cap must be what refuses it, not luck.
    st = Stub()
    for i in range(900):
        car_web.Console._drift_correct(st, _r(-0.10, 0.0, 0.95, 0.95), 0.0, i * 0.2 + 0.1)
    print("same residual, 0 m driven: applied %d, bias (%+.2f, %+.2f)"
          % (st.drift_n, st.car.bias[0], st.car.bias[1]))
    assert st.drift_n == 0 and st.car.bias == (0.0, 0.0), \
        "a stopped car cannot have drifted; the road has to pay for the correction"

    # ---- (2) closed loop on a synthetic room
    for rate in (0.03, 0.06):
        off = closed_loop(rate, False)
        on = closed_loop(rate, True)
        print("\nodometry gains %.0f cm per m, one %.1f m rectangle lap:" % (100 * rate, on["len"]))
        for name, r in (("corrector off", off), ("corrector on ", on)):
            print("  %s  err ends %.3f m  worst %.3f m  walls %.2f cells thick  occupied %d  "
                  "nudges %d (%.3f m)" % (name, r["final"], r["worst"], r["thick"], r["occ"],
                                          r["applied"], r["total"]))
        assert on["worst"] < off["worst"], "the suppressor did not reduce the pose error"
        assert on["thick"] < off["thick"], "the suppressor left the walls as doubled as they were"
        assert on["frozen"] == 0

    print("\nOK")


if __name__ == "__main__":
    main()
