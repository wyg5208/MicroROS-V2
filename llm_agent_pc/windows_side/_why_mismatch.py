# -*- coding: utf-8 -*-
"""Why did the matcher say "地图不匹配"? Replays the live sweep against the saved grid offline.

Usage: _why_mismatch.py            (needs the console up: it reads /api/state and /api/scan)

The frozen verdict is two numbers (zero=52%, best=83% at one offset) and neither says *which*
returns are responsible, or whether the winning offset is a structural match at all. Prints the
whole translation landscape as a picture, then the per-bearing residual at the pose odom claims
and at the pose the matcher prefers.
"""
import json
import math
import urllib.request

import numpy as np

import nav

URL = "http://127.0.0.1:8010"
REACH, STEP = 2.0, 0.10


def get(path):
    with urllib.request.urlopen(URL + path, timeout=10) as r:
        return json.loads(r.read().decode("utf-8"))


class Grid:
    """The bit of nav.World that the matcher touches."""

    def __init__(self, lo, res):
        self.lo, self.res = lo, res
        self.ox = -lo.shape[0] * res / 2.0
        self.oy = -lo.shape[0] * res / 2.0

    def grid(self):
        return self.lo


def score_map(world, pose, pairs, every=6):
    """score[row=dy][col=dx] with the same maths nav.scan_offset uses, over REACH at STEP."""
    lo = world.grid()
    occ = lo >= nav.MATCH_WALL
    dist = nav._dist_cap(occ, nav.DCAP)
    lut = np.exp(-(np.arange(nav.DCAP + 2) * world.res) ** 2 / (2.0 * nav.SIGMA ** 2))
    seen_d = nav._dilate(lo != 0.0)
    n = lo.shape[0]
    x, y, yaw = pose
    sel = list(pairs)[::every]
    th = np.radians(np.array([p[0] for p in sel])) + yaw
    rr = np.array([p[1] for p in sel])
    cx = (x + rr * np.cos(th) - world.ox) / world.res
    cy = (y + rr * np.sin(th) - world.oy) / world.res
    dm = np.arange(-REACH, REACH + 1e-9, STEP)
    dc = np.rint(dm / world.res).astype(np.int64)
    m = len(dm)
    gx = np.rint(cx[None, :] + dc[:, None]).astype(np.int64)
    gy = np.rint(cy[None, :] + dc[:, None]).astype(np.int64)
    okx = (gx >= 0) & (gx < n)
    oky = (gy >= 0) & (gy < n)
    inside = oky[:, None, :] & okx[None, :, :]
    ix = np.clip(gx, 0, n - 1)[None, :, :]
    iy = np.clip(gy, 0, n - 1)[:, None, :]
    w = lut[dist[iy, ix]]
    sup = seen_d[iy, ix] & inside
    nsup = sup.sum(2)
    score = np.where(nsup > 0, (w * sup).sum(2) / np.maximum(nsup, 1), 0.0)
    score[nsup < nav.MIN_SUPPORT] = 0.0
    hit = w * sup                                  # per (dy,dx,point) contribution
    return dm, score, hit, sup, nsup, sel


def bearing_view(world, pose, sel, off_y, off_x, tag):
    """Which directions the sweep disagrees with the map, at one candidate offset (metres)."""
    lo = world.grid()
    occ = lo >= nav.MATCH_WALL
    dist = nav._dist_cap(occ, nav.DCAP)
    n = lo.shape[0]
    x, y, yaw = pose
    dm_row, dm_col = off_y, off_x
    th = np.radians(np.array([p[0] for p in sel])) + yaw
    rr = np.array([p[1] for p in sel])
    gx = np.rint((x + dm_col + rr * np.cos(th) - world.ox) / world.res).astype(np.int64)
    gy = np.rint((y + dm_row + rr * np.sin(th) - world.oy) / world.res).astype(np.int64)
    ok = (gx >= 0) & (gx < n) & (gy >= 0) & (gy < n)
    d = np.full(len(sel), nav.DCAP + 1)
    d[ok] = dist[np.clip(gy, 0, n - 1)[ok], np.clip(gx, 0, n - 1)[ok]]
    bins = np.arange(-180, 181, 30)
    ang = np.array([p[0] for p in sel])
    # March the same rays through the grid: "what the map thinks is in that direction" next to
    # "what the lidar just measured" is the only comparison that says which of the two is lying.
    occ = lo >= nav.MATCH_WALL
    seen = lo != 0.0
    print("\n%s：按方位分档（回波中位距离 / 图上墙中位距离 / 端点离最近墙 / 点数）" % tag)
    print("  方位          实测    图上    离墙   n   判读")
    for a, b in zip(bins[:-1], bins[1:]):
        k = (ang >= a) & (ang < b)
        if k.sum() == 0:
            continue
        march = np.zeros(len(sel))
        todo = np.ones(len(sel), bool)
        for t in np.linspace(0.10, 6.0, 118):
            if not todo.any():
                break
            mx = x + dm_col + t * np.cos(th)
            my = y + dm_row + t * np.sin(th)
            c = np.floor((mx - world.ox) / world.res).astype(np.int64)
            r = np.floor((my - world.oy) / world.res).astype(np.int64)
            inb = (c >= 0) & (c < n) & (r >= 0) & (r < n)
            hitw = np.zeros(len(sel), bool)
            hitw[inb] = occ[r[inb], c[inb]]
            first = todo & hitw
            march[first] = t
            todo &= ~hitw
        md = np.median(march[k])
        print("  %+4d..%+4d°  %5.2f  %5.2f  %5.1f cm %3d  %s"
              % (a, b, np.median(rr[k]), md, np.median(d[k]) * world.res * 100, int(k.sum()),
                 "" if (np.median(d[k]) <= 2 and abs(md - np.median(rr[k])) < 0.25)
                 else ("图上无墙" if md <= 0.11 else "差 %+.2f m" % (np.median(rr[k]) - md)))
              )


def main():
    st = get("/api/state")
    sc = get("/api/scan")
    lo = np.load("cache_console/grid.npy")
    ox, oy = json.load(open("cache_console/origin.json"))
    world = Grid(lo, 0.05)
    pose = (st["pose"][0] + ox, st["pose"][1] + oy, math.radians(st["pose"][2]))
    pairs = [(float(a), float(r)) for a, r in sc["pairs"]]
    print("占据格 %d  位姿 (%+.2f, %+.2f, %+.1f°)  回波 %d 点  雷达年龄 %.2f s  建图 %s  冻结 %s"
          % (int((lo >= nav.MATCH_WALL).sum()), pose[0], pose[1], math.degrees(pose[2]),
             len(pairs), sc["age"], st["mapping"], st["reloc"]["frozen"]))

    dm, score, hit, sup, nsup, sel = score_map(world, pose, pairs)
    m = len(dm)
    z = m // 2
    i = int(np.argmax(score))
    br, bc = i // m, i % m
    print("最优 %.3f @ (%+.2f, %+.2f)   零偏移 %.3f   支撑 %d/%d 点"
          % (score[br, bc], dm[bc], dm[br], score[z, z], int(nsup[br, bc]), len(sel)))
    print("\n平移吻合度地形（数字 = 该偏移下整扫的吻合 %%; 行 dy 自上而下 +2.0 → -2.0，列 dx -2.0 → +2.0）")
    print("      " + "".join("%4.0f" % (v * 100) for v in dm[::2]))
    for r in range(m - 1, -1, -1):
        if abs(dm[r] - round(dm[r])) < 1e-9:
            print("%+5.1f " % dm[r] + "".join("%4d" % (score[r, c] * 100) for c in range(0, m, 2)))
    print("      " + "".join("%4s" % ("↑零" if abs(v) < 1e-9 else "") for v in dm[::2]))

    bearing_view(world, pose, sel, dm[z], dm[z], "里程计位姿（零偏移）")
    bearing_view(world, pose, sel, dm[br], dm[bc],
                 "匹配器首选 (%+.2f, %+.2f)" % (dm[bc], dm[br]))


if __name__ == "__main__":
    main()
