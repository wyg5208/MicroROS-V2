"""Offline check of the per-sign angular fit + the voltage scale. Standard library only.

Drives car_web.fit() the same way /api/fit does, with samples built to have a known answer
(left and right fitted apart, then compared against the constants the car would use).
"""
import json
import math
import sys
import types

sys.path.insert(0, __import__("os").path.dirname(__file__))
import nav  # noqa: E402


def fake(rows):
    ns = types.SimpleNamespace()
    ns.samples = list(rows)
    ns.push = lambda *a: None
    ns.car = types.SimpleNamespace(volts=None)
    return ns


def check(name, got, want, tol=0.02):
    ok = abs(got - want) <= tol
    print("%-28s got %.4f want %.4f  %s" % (name, got, want, "OK" if ok else "FAIL"))
    return ok


fails = []

# truth: ccw |w_act| = 0.80|w| - 0.10 (dead 0.125), cw 0.62|w| - 0.20 (dead 0.323), linear 0.85.
# Commands stay above their own dead zone: an achieved rate is a measured sign-and-magnitude, and
# a below-dead-zone sample really reads ~0, which no straight line can also fit.
rows = []
for w in (0.15, 0.30, 0.50, 0.75):
    rows.append((0.0, w, 0.0, 0.80 * w - 0.10, 4.0, "calib"))
for w in (-0.40, -0.55, -0.70, -0.75):
    rows.append((0.0, w, 0.0, -(0.62 * abs(w) - 0.20), 4.0, "calib"))
for v in (0.06, 0.10, 0.12):
    rows.append((v, 0.0, 0.85 * v, 0.0, 6.0, "teleop"))

import car_web  # noqa: E402

out = car_web.Console.fit(fake(rows), apply=False)
print(json.dumps({k: v for k, v in sorted(out.items())}, ensure_ascii=False, default=str))
fails += [not check("n_ccw", out["n_ccw"], 4, 0), not check("n_cw", out["n_cw"], 4, 0)]
fails += [not check("split fit with 4+4 samples", bool(out.get("split", False)), True)]

fails += [not check("gain_w (ccw)", out["gain_w"], 0.80),
          not check("w_bias (ccw)", out["w_bias"], 0.10),
          not check("w_dead (ccw)", out["w_dead"], 0.125),
          not check("gain_w_neg (cw)", out["gain_w_neg"], 0.62),
          not check("w_bias_neg (cw)", out["w_bias_neg"], 0.20),
          not check("gain_v", out["gain_v"], 0.85)]

# one side missing -> mirror, not a two-point line
rows1 = [r for r in rows if r[1] >= 0.0]
o1 = car_web.Console.fit(fake(rows1), apply=False)
fails += [not check("mirror cw==ccw gain", o1["gain_w_neg"], o1["gain_w"], 1e-9)]

# too few of both -> pooled over whatever there is, no independent claim per side
o2 = car_web.Console.fit(fake(rows[:2]), apply=False)
fails += [not check("no split claim when a side is short", bool(o2.get("split", False)), False)]

# duration gate: the live pool that triggered this was 19 teleop bursts averaging the ramp
# phase (a -0.24 rad/s command reporting -0.040), which fit to gain_w 0.015. Short rows must
# be invisible to the fit, not merely diluted by it.
short = []
for w in (-0.15, -0.24, -0.35, -0.50, -0.60, 0.20, 0.45):
    short.append((0.0, w, 0.0, 0.15 * w, 1.0, "teleop"))       # never reached speed
for v in (0.08, 0.12):
    short.append((v, 0.0, 0.20 * v, 0.0, 1.2, "teleop"))
mixed = rows + short
o3 = car_web.Console.fit(fake(mixed), apply=False)
fails += [not check("dropped short samples", o3["n_short_dropped"], 9, 0),
          not check("kept samples counted", o3["n_total"], 20, 0),
          not check("gate leaves ccw count", o3["n_ccw"], 4, 0),
          not check("gate leaves gain_w", o3["gain_w"], 0.80),
          not check("gate leaves gain_w_neg", o3["gain_w_neg"], 0.62),
          not check("gate leaves gain_v", o3["gain_v"], 0.85)]
o4 = car_web.Console.fit(fake(short), apply=False)
fails += [not check("all-short pool has no linear", o4["n_linear"], 0, 0),
          not check("all-short pool has no angular", o4["n_angular"], 0, 0),
          not check("all-short pool makes no claim", "gain_w" in o4, False)]

# achieved() sign selection: each half has to use its own gain, bias and dead zone
nav.ACT_GAIN_W, nav.ACT_W_BIAS, nav.ACT_W_DEAD = 0.80, 0.10, 0.125
nav.ACT_GAIN_W_NEG, nav.ACT_W_BIAS_NEG, nav.ACT_W_DEAD_NEG = 0.62, 0.20, 0.323
fails += [not check("achieved ccw +0.50", nav.achieved(0.0, 0.50)[1], 0.30),
          not check("achieved cw -0.50", nav.achieved(0.0, -0.50)[1], -0.11),
          not check("achieved cw -0.25 (under dead)", nav.achieved(0.0, -0.25)[1], 0.0),
          not check("achieved ccw +0.20 (over dead)", nav.achieved(0.0, 0.20)[1], 0.06),
          not check("achieved ccw +0.10 (under dead)", nav.achieved(0.0, 0.10)[1], 0.0)]

# voltage scale: gains yes, stiction no
nav.ACT_VOLT_REF, nav.ACT_VOLTS = 12.0, 12.0
base_v, base_w = nav.achieved(0.10, 0.50)
nav.ACT_VOLTS = 10.0                                     # 0.833 -> clamped only at 0.70
s_v, s_w = nav.achieved(0.10, 0.50)
k = 10.0 / 12.0
fails += [not check("volt-scaled v", s_v, base_v * k),
          not check("volt-scaled w", s_w, base_w * k)]
nav.ACT_VOLTS = 5.0                                      # clamped at the floor
fails += [not check("volt clamp floor", nav.volt_scale(), 0.70)]
nav.ACT_VOLTS = 20.0
fails += [not check("volt clamp ceil", nav.volt_scale(), 1.10)]
nav.ACT_VOLTS = None
fails += [not check("volt unknown", nav.volt_scale(), 1.0)]

# old nav_gains.json (no per-side keys) must not leave the cw half at the factory constant
import tempfile  # noqa: E402, F401
import os  # noqa: E402
old = {"gain_v": 0.9, "gain_w": 0.7, "w_bias": 0.2, "w_dead": 0.3, "max_v": 0.12, "max_w": 0.75}
path = os.path.join(tempfile.mkdtemp(), "nav_gains.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(old, f)
car_web.GAINS_FILE = path
fails += [not car_web.load_gains()]
fails += [not check("legacy gain_w_neg", nav.ACT_GAIN_W_NEG, 0.7, 1e-9),
          not check("legacy w_dead_neg", nav.ACT_W_DEAD_NEG, 0.3, 1e-9)]
print(json.dumps(car_web.gains(), ensure_ascii=False, indent=1))

print("FAILURES: %d" % len([x for x in fails if x]))
print("PASS" if not any(fails) else "FAIL")
