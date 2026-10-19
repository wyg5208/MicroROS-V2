"""Time the planner's per-tick work so the cmd_vel stream can be kept at 20 Hz.

The board wants a continuous stream (CMD_PERIOD 0.05 s). A 360-degree MapSpin commanded at
0.35 rad/s turned the car only 67 degrees, which smells like the loop publishing every ~0.2 s
because the mapping step is expensive. This measures each step in isolation at realistic
input sizes.
"""
import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

import nav

N = 667
RNG = 8.0


def fake_scan(seed=1):
    rnd = random.Random(seed)
    pairs = []
    for i in range(N):
        a = math.radians(i * 360.0 / N)
        d = rnd.uniform(0.3, RNG) if rnd.random() > 0.15 else 0.0
        pairs.append((a, d))
    return pairs


def main():
    world = nav.World()
    pose = (2.0, 6.0, 0.5)
    scans = [fake_scan(i) for i in range(40)]

    t0 = time.perf_counter()
    for i in range(200):
        world.integrate(pose, scans[i % 40], every=3, samples=12)
    dt_int = (time.perf_counter() - t0) / 200

    cm = nav.Costmap(world.grid(), world.res, world.ox, world.oy)
    t0 = time.perf_counter()
    for i in range(200):
        cm.rebuild(pose, scans[i % 40])
    dt_reb = (time.perf_counter() - t0) / 200

    t0 = time.perf_counter()
    for i in range(20):
        nav.Costmap(world.grid(), world.res, world.ox, world.oy)
    dt_ctor = (time.perf_counter() - t0) / 20

    t0 = time.perf_counter()
    for i in range(200):
        nav.steer(cm, pose, (pose[0] + 1.0, pose[1] + 0.3))
    dt_steer = (time.perf_counter() - t0) / 200

    goal = (pose[0] + 1.2, pose[1] + 0.8)
    t0 = time.perf_counter()
    ok = 0
    for i in range(30):
        p = nav.astar(cm, (pose[0], pose[1]), goal)
        ok += 1 if p else 0
    dt_astar = (time.perf_counter() - t0) / 30

    t0 = time.perf_counter()
    for i in range(30):
        nav.global_frontier(world, pose)
    dt_gf = (time.perf_counter() - t0) / 30

    print("integrate      %6.1f ms" % (dt_int * 1e3))
    print("Costmap() 新建 %6.1f ms" % (dt_ctor * 1e3))
    print("rebuild        %6.1f ms" % (dt_reb * 1e3))
    print("steer          %6.1f ms" % (dt_steer * 1e3))
    print("astar          %6.1f ms (%d/30 成功)" % (dt_astar * 1e3, ok))
    print("global_frontier %6.1f ms" % (dt_gf * 1e3))
    print()
    print("每 tick = integrate/4 + 新建 + rebuild + steer = %.1f ms → 可发布 %.1f Hz"
          % ((dt_int / 4 + dt_ctor + dt_reb + dt_steer) * 1e3,
             1.0 / max(1e-6, dt_int / 4 + dt_ctor + dt_reb + dt_steer)))
    print("规划一次 = astar + global_frontier = %.1f ms" % ((dt_astar + dt_gf) * 1e3))


main()
