# -*- coding: utf-8 -*-
"""What does the map-to-lidar residual actually look like while the car is moving?

The carry detector answers a metres-scale question from a stopped car, and there the sweep
matches the grid at ~0.99. The drift question is centimetres, so before any correction can be
tuned, the noise floor of the *moving* measurement has to be measured: how much of the residual
is signal (growing with distance travelled) and how much is a flat search surface picking an
arbitrary peak.

Reads /api/drift from a running bridge (rows: t, dx, dy, zero, best, support, metres moved,
heading) and reports the two numbers a deadband and an update gain are chosen from.
"""
import json
import math
import statistics as S
import sys
import urllib.request

URL = "http://127.0.0.1:8010/api/drift"


def slope(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    var = sum((x - mx) ** 2 for x in xs) / n
    cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / n
    sd_x = math.sqrt(var) or 1e-9
    sd_y = math.sqrt(sum((y - my) ** 2 for y in ys) / n) or 1e-9
    return cov / var, cov / (sd_x * sd_y)


def main():
    rows = json.load(urllib.request.urlopen(URL, timeout=8))["hist"]
    if len(sys.argv) > 1:                      # optional: ignore nearly-stationary samples
        rows = [r for r in rows if r[6] > float(sys.argv[1])]
    print("samples %d, %.2f m travelled while sampled" % (len(rows), sum(r[6] for r in rows)))
    for gate in (0.0, 0.85, 0.90, 0.95):
        keep = [r for r in rows if r[3] >= gate]
        if len(keep) < 5:
            print("zero >= %.2f: %d samples (too few)" % (gate, len(keep)))
            continue
        dx = [r[1] for r in keep]
        dy = [r[2] for r in keep]
        cum, acc = [], 0.0
        for r in keep:
            acc += r[6]
            cum.append(acc)
        sx, rx = slope(cum, dx)
        sy, ry = slope(cum, dy)
        n = len(keep)
        # The number a deadband must clear: the scatter of an average of n samples. If drift is
        # real it shows up as slope (m per m travelled); if it is noise the slope is meaningless
        # and this SE is the floor below which any "correction" would be chasing phantoms.
        se_x, se_y = S.pstdev(dx) / math.sqrt(n), S.pstdev(dy) / math.sqrt(n)
        print("zero >= %.2f: n=%3d  dx %+0.3f±%.3f (SE %.3f) slope %+0.4f m/m corr %+.2f | "
              "dy %+0.3f±%.3f (SE %.3f) slope %+0.4f m/m corr %+.2f"
              % (gate, n, S.mean(dx), S.pstdev(dx), se_x, sx, rx,
                 S.mean(dy), S.pstdev(dy), se_y, sy, ry))
    best_gain = [r[4] - r[3] for r in rows]
    print("best-zero: median %.3f  p90 %.3f  (>0.10 means the sweep really belongs elsewhere)"
          % (S.median(best_gain), sorted(best_gain)[int(0.9 * len(best_gain))]))


if __name__ == "__main__":
    main()
