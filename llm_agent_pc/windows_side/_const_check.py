"""Offline arithmetic for the turn constants, against nav_gains.json and the logged sweeps.

Why this exists: the turn floor and the heading hold both have to sit above what the chassis
executes. The fitted `w_dead` is where the least-squares *line* crosses zero, which is not the same
number as the command the wheels stop obeying -- so this prints both, from the real rows, and moves
nothing.

    _rosrun.bat _const_check.py
"""
import json
import math
import os

import nav

HERE = os.path.dirname(os.path.abspath(__file__))
FALLBACK = {"gain_v": 0.82, "gain_w": 0.857, "gain_w_neg": 0.857,
            "w_bias": 0.101, "w_bias_neg": 0.101, "w_dead": 0.118, "w_dead_neg": 0.118,
            "volt_ref": 12.0}
SPEED = 1.0                 # --angular-speed default: what the full-speed part of a turn is


def load():
    try:
        with open(os.path.join(HERE, "nav_gains.json"), encoding="utf-8") as f:
            g = json.load(f)
    except (OSError, ValueError):
        return dict(FALLBACK)
    out = dict(FALLBACK)
    out.update({k: v for k, v in g.items() if k in out and v is not None})
    return out


def show(g):
    nav.ACT_GAIN_V = g["gain_v"]
    nav.ACT_GAIN_W = g["gain_w"]
    nav.ACT_GAIN_W_NEG = g["gain_w_neg"]
    nav.ACT_W_BIAS = g["w_bias"]
    nav.ACT_W_BIAS_NEG = g["w_bias_neg"]
    nav.ACT_W_DEAD = g["w_dead"]
    nav.ACT_W_DEAD_NEG = g["w_dead_neg"]
    nav.ACT_VOLT_REF = g["volt_ref"]
    nav.ACT_VOLTS = None


def rows():
    try:
        with open(os.path.join(HERE, "calib_samples.jsonl"), encoding="utf-8") as f:
            return [json.loads(l) for l in f if l.strip()]
    except (OSError, ValueError):
        return []


def main():
    g = load()
    show(g)
    dead_max = max(g["w_dead"], g["w_dead_neg"])
    print("fit: gain_w %.3f/%.3f  bias %.3f/%.3f  死区(拟合) %.3f/%.3f  volt_ref %.2f"
          % (g["gain_w"], g["gain_w_neg"], g["w_bias"], g["w_bias_neg"],
             g["w_dead"], g["w_dead_neg"], g["volt_ref"]))
    print("实测起转下限 ACT_W_CUTON = %.2f → 有效死区 %.2f/%.2f，nav.turn_floor() = %.3f rad/s"
          % (nav.ACT_W_CUTON, max(g["w_dead"], nav.ACT_W_CUTON),
             max(g["w_dead_neg"], nav.ACT_W_CUTON), nav.turn_floor()))
    nav.ACT_VOLTS = 10.3
    print("  该命令在满压交付 %.1f°/s(左) %.1f°/s(右)；掉到 10.3 V 低压锁边缘仍是 %.1f/%.1f°/s"
          % (math.degrees(abs(nav.achieved(0.0, nav.turn_floor())[1])) / min(1.0, 10.3 / g["volt_ref"]),
             math.degrees(abs(nav.achieved(0.0, -nav.turn_floor())[1])) / min(1.0, 10.3 / g["volt_ref"]),
             math.degrees(abs(nav.achieved(0.0, nav.turn_floor())[1])),
             math.degrees(abs(nav.achieved(0.0, -nav.turn_floor())[1]))))
    show(g)

    r = rows()
    ang = [c for c in r if abs(c[0]) < 0.02 and abs(c[1]) >= 0.05 and c[4] >= 2.5]
    if ang:
        print("\n== 日志里的转向样本：拟合预测 vs 实测（正=逆时针）==")
        print("%7s %9s %9s %8s %6s" % ("指令", "预测", "实测", "差", "电压"))
        for c in sorted(ang, key=lambda x: abs(x[1])):
            w, act = c[1], c[3]
            k = 1.0 if not c[6] else max(0.70, min(1.10, c[6] / g["volt_ref"]))
            nav.ACT_VOLTS = None if k == 1.0 else g["volt_ref"] * k
            pred = nav.achieved(0.0, w)[1]
            nav.ACT_VOLTS = None
            flag = "  ← 命令在起转下限之下" if abs(w) <= nav.ACT_W_CUTON else ""
            print("%+7.2f %+9.4f %+9.4f %+8.4f %6s%s" % (
                w, pred, act, act - pred, ("%.2f" % c[6]) if c[6] else "-", flag))

    print("\n== 收尾律会不会自己掉进不执行的区间（speed=%.1f，释放容差同 _closed_loop）==" % SPEED)
    for deg in (15, 30, 45, 90):
        need = math.radians(deg)
        tol = max(0.02, 0.04 * need)
        prop = SPEED * tol / need * 2.5
        cmd = max(prop, nav.turn_floor())
        print("  %2d°：容差 %.2f°，比例律到容差时给 %.3f，取下限后 %.3f → 交付 %.1f°/s，"
              "末段爬 %.2f s" % (deg, math.degrees(tol), prop, cmd,
                                 math.degrees(abs(nav.achieved(0.0, cmd)[1])),
                                 math.degrees(tol) / max(
                                     math.degrees(abs(nav.achieved(0.0, cmd)[1])), 1e-9)))
    print("\n== 不动点解释力（旧拟合 vs 现拟合，45° 那条实测短 2.04°）==")
    for tag, gg in (("旧拟合 dead 0.118", FALLBACK), ("现拟合", g)):
        show(gg)
        dead = max(gg["w_dead"], gg["w_dead_neg"], nav.ACT_W_CUTON)
        stuck = math.degrees(dead * math.radians(45.0) / (2.5 * SPEED))
        print("  %s：指令掉进不执行区那一刻还剩 %.2f° → %s" % (
            tag, stuck, "解释得通（实测 2.04°）" if abs(stuck - 2.04) < 0.6 else "解释不了"))
    show(g)


if __name__ == "__main__":
    main()
