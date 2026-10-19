"""Simulate the tour controller against a synthetic room -- no car, no WiFi, no risk.

Usage: _sim.py [seconds]

Why this exists: the wall-follower it checks covered metres and still drove into a person's
feet, and the only way to tell "the mechanism reasons about the room" from "the mechanism got
lucky" before spending another floor test is to run it against a room whose geometry is known
exactly. The lidar model reproduces this board's real defects rather than an ideal one: the
-24..+20 deg gimbal blind wedge is missing entirely (not faked as a short return), because
that is what the bridge actually hands the planner.
"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav

BLIND = (-24.0, 20.0)
RAY_STEP = 3              # every Nth of 667 rays; 1 s of sim should not take a minute


def segments(walls):
    return [((a[0], a[1]), (b[0], b[1])) for a, b in walls]


def cast(x, y, ang, segs, rmax=8.0):
    dx, dy = math.cos(ang), math.sin(ang)
    best = rmax
    for (ax, ay), (bx, by) in segs:
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((ax - x) * ey - (ay - y) * ex) / den
        u = ((ax - x) * dy - (ay - y) * dx) / den
        if 0 <= t <= rmax and 0.0 <= u <= 1.0:
            best = min(best, t)
    return best


def scan(pose, segs):
    x, y, yaw = pose
    out = []
    for i in range(0, 667, RAY_STEP):
        a = -180.0 + i * 360.0 / 667.0
        if BLIND[0] <= a <= BLIND[1]:
            continue
        r = cast(x, y, yaw + math.radians(a), segs)
        if r >= 7.9 or r <= 0.15:
            continue
        out.append((a, r))
    return out


def room():
    """A 5.2 x 6.4 m room with a table, a cabinet and a chair -- the shape of the real one."""
    x0, y0, x1, y1 = -2.6, -1.6, 2.6, 4.8
    outer = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
             ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    def rect(cx, cy, w, h):
        a, b = (cx - w / 2, cy - h / 2), (cx + w / 2, cy + h / 2)
        return [((a[0], a[1]), (b[0], a[1])), ((b[0], a[1]), (b[0], b[1])),
                ((b[0], b[1]), (a[0], b[1])), ((a[0], b[1]), (a[0], a[1]))]
    return outer + rect(-1.4, 1.2, 0.9, 0.9) + rect(1.9, 2.6, 1.2, 0.6) + rect(0.2, -0.6, 0.5, 0.5)


def run(budget=200.0, start=(-0.0, 3.6, math.radians(-81.0)), dt=0.05, trace=None, diag=False):
    """Fly the real Pilot through a synthetic room whose lidar has the same blind wedge as
    the hardware. Every decision below happens inside nav.Pilot, i.e. this exercises the code
    that runs on the car rather than a copy of it."""
    segs = room()
    x, y, yaw = start
    x0, y0 = x, y
    world = nav.World()
    pilot = nav.Pilot(world, (x0, y0))
    events = []
    t = 0.0
    dist = 0.0
    hits = []
    steps = int(budget / dt)
    t_wall = time.monotonic()

    def result(reason):
        return dict(reason=reason, t=t, dist=dist, x=x, y=y, yaw=math.degrees(yaw),
                    home=math.hypot(x - x0, y - y0), frames=world.frames,
                    occ=int((world.grid() >= 0.65).sum()), stalls=pilot.stalls,
                    recovers=pilot.recovers, noreplan=pilot.noreplan, hits=hits,
                    events=events, secs=time.monotonic() - t_wall)

    for _ in range(steps):
        pose = (x, y, yaw)
        pairs = scan(pose, segs)
        stalls0 = pilot.stalls
        v, w = pilot.step(t, pose, pairs)
        cm = pilot.cm
        if pilot.stalls > stalls0:
            events.append("卡死 t=%5.1f (%5.2f,%5.2f) yaw%+4d %s 目标(%5.2f,%5.2f) "
                          "本体代价%.2f 空闲格%d/%d"
                          % (t, x, y, round(math.degrees(yaw)), pilot.src,
                             pilot.goal[0], pilot.goal[1], cm.at(x, y),
                             int((cm.cost < nav.COST_BLOCK).sum()), cm.cost.size))
        if diag and int(t * 2) % 8 == 0:
            print("t=%5.1f (%5.2f,%5.2f) yaw%+4d %s 目标(%5.2f,%5.2f) d=%.2f 起点代价%.2f "
                  "路径%s v=%+.2f w=%+.2f"
                  % (t, x, y, round(math.degrees(yaw)), pilot.src,
                     pilot.goal[0], pilot.goal[1],
                     math.hypot(pilot.goal[0] - x, pilot.goal[1] - y), cm.at(x, y),
                     "-" if pilot.path is None else len(pilot.path), v, w))
        v, w = nav.achieved(v, w)   # the board gives back ~90% and ignores small wz
        yaw2 = yaw + w * dt
        nx = x + v * dt * math.cos(yaw2)
        ny = y + v * dt * math.sin(yaw2)
        # collision truth: distance from the new centre to every real wall segment
        d = min(_pt_seg(nx, ny, a, b) for a, b in segs)
        if d < nav.CHASSIS_R:
            hits.append((t, x, y, d))
            break
        dist += math.hypot(nx - x, ny - y)
        x, y, yaw = nx, ny, yaw2
        if trace is not None:
            trace.append((t, x, y, math.degrees(yaw)))
        t += dt
        if t > 25.0 and dist > 4.0 and math.hypot(x - x0, y - y0) < 0.5:
            return result("回到出发点")
    return result("时间到/撞停" if hits else "时间到")


def _pt_seg(px, py, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    u = 0.0 if L == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(px - (ax + u * dx), py - (ay + u * dy))


if __name__ == "__main__":
    tr = []
    r = run(float(sys.argv[1]) if len(sys.argv) > 1 else 200.0, trace=tr)
    print("结论 %s" % r["reason"])
    print("仿真 %.0f s 用真实时间 %.1f s" % (r.get("t", 0), r.get("secs", 0)))
    print("走了 %.2f m，终点 (%.2f, %.2f)，离出发点 %.2f m" % (
        r["dist"], r["x"], r["y"], r["home"]))
    print("建图 %d 帧 / 占据 %d 格，倒车脱困 %d 次，卡死重启 %d 次，规划失败 %d 次" % (
        r["frames"], r["occ"], r["recovers"], r["stalls"], r["noreplan"]))
    for e in r["events"]:
        print("  " + e)
    print("碰撞 %d 次" % len(r["hits"]))
    if r["hits"]:
        print("  第一次：t=%.1f s 在 (%.2f, %.2f)，离墙 %.3f m" % r["hits"][0])
    import json
    json.dump([[round(v, 3) for v in p] for p in tr],
              open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_trace.json"), "w"))
    print("轨迹已存 sim_trace.json（%d 点）" % len(tr))
