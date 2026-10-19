# -*- coding: utf-8 -*-
"""Pick DRIFT_ALPHA / DRIFT_DEAD from measurements, not from taste.

The two numbers have to be settled against three different signals at once:
  * a +-9 cm alternating wobble (pure measurement noise) -- must pay out nothing;
  * the 21 moving samples the real car produced -- should stay quiet, it was well registered;
  * a synthetic lap whose odometry gains 3 cm per metre -- should engage and sharpen the map.
A deadband that is too tight fails the first two, one that is too loose fails the third.
"""
import math

import car_web
import _drift_test as dt


def wobble(alpha, dead):
    st = dt.Stub()
    for i in range(400):
        car_web.Console._drift_correct(st, dt._r(0.09 if i % 2 else -0.09, 0.0, 0.95, 0.95),
                                      0.04, i * 0.2 + 0.1)
    return st.drift_n, math.hypot(*st.car.bias)


def replay(alpha, dead):
    st = dt.feed(dt.REAL)
    return st.drift_n, math.hypot(*st.car.bias)


def main():
    print("%-14s %-22s %-22s %s" % ("alpha / dead", "+-9 cm wobble (m driven)",
                                    "real 21 samples (0.85 m)", "synthetic lap, 3 cm/m"))
    for alpha in (0.15, 0.25, 0.40):
        for dead in (0.02, 0.03, 0.04):
            car_web.DRIFT_ALPHA, car_web.DRIFT_DEAD = alpha, dead
            wn, wb = wobble(alpha, dead)
            rn, rb = replay(alpha, dead)
            lap = dt.closed_loop(0.03, True)
            print("%.2f / %.2f      n=%-3d bias %.3f m      n=%-3d bias %.3f m      "
                  "worst %.3f m  walls %.2f cells  n=%d"
                  % (alpha, dead, wn, wb, rn, rb, lap["worst"], lap["thick"], lap["applied"]))
    car_web.DRIFT_ALPHA, car_web.DRIFT_DEAD = 0.25, 0.04
    off = dt.closed_loop(0.03, False)
    print("(no corrector at all:  worst %.3f m  walls %.2f cells)" % (off["worst"], off["thick"]))


if __name__ == "__main__":
    main()
