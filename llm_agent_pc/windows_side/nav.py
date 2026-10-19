"""Map + lidar fused local planning for the MicroROS-V2 chassis.

Imported by car_mcp_server.py -- it must run inside that one long-lived process because a
freshly created rclpy node receives nothing from the board (confirmed three times: the bridge
shows /scan 0.0 s fresh while standalone nodes log 0 frames).

Three layers, deliberately separated so each can be judged on its own:

  Costmap   an odom-frame window re-cut around the car. Base layer = the saved occupancy grid.
            Live layer = the *current* lidar ring splatted on top, rebuilt from scratch every
            tick, so a person who walked into the corridor is an obstacle now and not at all
            once they leave. The two are combined with max(): the lidar may add obstacles, and
            a stale map may never delete one it already knows about.
  astar     8-connected search over that costmap, obstacles as traversal penalty.
  steer     Dynamic-window trajectory scoring: simulate candidate (v, w) arcs, throw out the
            ones that touch an obstacle, rank the rest by progress + clearance. Reversing is
            a legal candidate, which is the whole point -- a controller that cannot back off
            will press a wheel into whatever it walked into.
"""
import heapq
import math
from collections import deque

import numpy as np

RES = 0.05                 # metres per cell, matches mapper.py's fine setting
WIN = 4.0                  # planning window edge, metres
SIDE = int(WIN / RES)      # cells per side
CHASSIS_R = 0.11           # m, the disc the footprint is approximated by
INFLATE = 0.07             # extra margin beyond the disc
COST_BLOCK = 0.85
COST_UNKNOWN = 0.45        # never measured: soft obstacle, not a wall and not open floor
MAX_V = 0.12               # m/s, commanded
MAX_W = 0.75               # rad/s, commanded

# What the board does with a command is not the command. A 12-burst RateTest sweep on this
# chassis (2026-10-18) least-squares fits to gain_v 0.82 (RMSE 0.002 m/s) and
# w_act = 0.857*w_cmd - 0.101*sign(w) (RMSE 0.011 rad/s), i.e. nothing at all below
# 0.118 rad/s and only 0.41 rad/s out of the vendor's 0.6 ceiling. Predicting arcs from the
# command instead of the response makes the planner score trajectories the car cannot follow.
#
# Left and right are separate fits. The vendor's own calibration sheet ships different scale
# factors for the two directions (left 1.0708 / right 1.0863), which is what you expect from
# two motors with two PWM paths, and the *_NEG constants carry the clockwise half. They start
# out equal to the counter-clockwise ones so behaviour is unchanged until a fit says otherwise.
ACT_GAIN_V = 0.82
ACT_GAIN_W = 0.857
ACT_W_BIAS = 0.101
ACT_W_DEAD = 0.118
ACT_GAIN_W_NEG = ACT_GAIN_W
ACT_W_BIAS_NEG = ACT_W_BIAS
ACT_W_DEAD_NEG = ACT_W_DEAD

# Same battery, same command, less speed. The board drives 12 V motors off a 3S pack, so the
# achievable duty -- and with it the whole response -- shrinks as the pack sags. /battery
# republishes the fitted voltage into ACT_VOLTS and the gains are scaled by the ratio to the
# voltage they were fitted at. Stiction (the bias) is deliberately left alone: it is a fixed
# PWM floor in the firmware, not something that scales with supply.
ACT_VOLT_REF = 12.0
ACT_VOLTS = None
VOLT_CLAMP = (0.70, 1.10)


def volt_scale():
    """Gain multiplier from the pack voltage the car is running on right now."""
    if not ACT_VOLTS or not ACT_VOLT_REF:
        return 1.0
    return max(VOLT_CLAMP[0], min(VOLT_CLAMP[1], ACT_VOLTS / ACT_VOLT_REF))


def achieved(v, w):
    """A commanded twist -> the twist the chassis actually delivers."""
    k = volt_scale()
    if w < 0.0:
        gain, bias, dead = ACT_GAIN_W_NEG, ACT_W_BIAS_NEG, ACT_W_DEAD_NEG
    else:
        gain, bias, dead = ACT_GAIN_W, ACT_W_BIAS, ACT_W_DEAD
    if abs(w) < dead:
        w = 0.0
    else:
        w = math.copysign(max(0.0, gain * abs(w) - bias), w)
    return ACT_GAIN_V * k * v, w * k


# ---------------------------------------------------------------- costmap ----
class Costmap:
    """Sliding odom-frame window: saved grid underneath, live lidar ring on top."""

    def __init__(self, grid=None, res=RES, ox=0.0, oy=0.0):
        self.base = grid            # float32 log-odds from grid.npy, or None
        self.res = res
        self.ox = ox
        self.oy = oy
        self.origin = (0.0, 0.0)    # odom coords of this window's cell (0,0)
        self.cost = np.zeros((SIDE, SIDE), np.float32)
        self.has_base = grid is not None

    def _base_cost(self, xs, ys):
        """Saved grid sampled at world points -> cost in [0,1]; outside the map = unknown."""
        if not self.has_base:
            return np.full(xs.shape, COST_UNKNOWN, np.float32)
        c = np.floor((xs - self.ox) / self.res).astype(np.int64)
        r = np.floor((ys - self.oy) / self.res).astype(np.int64)
        n = self.base.shape[0]
        ok = (c >= 0) & (c < n) & (r >= 0) & (r < n)
        lo = np.where(ok, self.base[np.clip(r, 0, n - 1), np.clip(c, 0, n - 1)], 0.0)
        out = np.full(xs.shape, COST_UNKNOWN, np.float32)
        out[ok & (lo <= -2.0)] = 0.05          # confidently free
        out[ok & (lo >= 0.65)] = 1.0            # occupied
        # exactly 0.0 means no ray ever crossed it -- both World and mapper.py leave it at
        # zero, so folding it into the band below would make the unmapped area look mildly
        # penalised-but-known and the frontier search would find no frontier at all
        out[ok & (lo > -2.0) & (lo < 0.65) & (lo != 0.0)] = 0.35
        return out

    def rebuild(self, pose, pairs):
        """pose = (x, y, yaw) in odom; pairs = usable body-frame (deg, range) lidar returns."""
        x, y, yaw = pose[0], pose[1], pose[2]
        # centre the window on the car but keep it on whole cells so a stationary car does not
        # re-cut the grid by a sub-cell amount every tick
        self.origin = (x - WIN / 2.0, y - WIN / 2.0)
        cols = np.arange(SIDE)
        wx = self.origin[0] + (cols + 0.5) * RES
        wy = self.origin[1] + (cols + 0.5) * RES
        gx, gy = np.meshgrid(wx, wy)           # row = y index, same as mapper's grid
        self.cost = self._base_cost(gx, gy)

        if pairs:
            th = np.radians(np.array([p[0] for p in pairs])) + yaw
            rr = np.array([p[1] for p in pairs])
            px = x + rr * np.cos(th)
            py = y + rr * np.sin(th)
            self._stamp(px, py, 1.0)
            # The ray interiors are being seen through *right now*. Lower a mapped obstacle to
            # a penalty rather than deleting it: a wall the lidar reports through is moved
            # furniture, and "probably fine" is the honest reading, not "definitely open".
            for t in (0.35, 0.6, 0.85):
                self._lower(x + t * rr * np.cos(th), y + t * rr * np.sin(th), 0.3)

        self._inflate()
        return self

    def _stamp(self, px, py, value):
        c = np.floor((px - self.origin[0]) / RES).astype(np.int64)
        r = np.floor((py - self.origin[1]) / RES).astype(np.int64)
        ok = (c >= 0) & (c < SIDE) & (r >= 0) & (r < SIDE)
        np.maximum.at(self.cost, (r[ok], c[ok]), np.float32(value))

    def _lower(self, px, py, value):
        c = np.floor((px - self.origin[0]) / RES).astype(np.int64)
        r = np.floor((py - self.origin[1]) / RES).astype(np.int64)
        ok = (c >= 0) & (c < SIDE) & (r >= 0) & (r < SIDE)
        self.cost[r[ok], c[ok]] = np.minimum(self.cost[r[ok], c[ok]], value)

    def _inflate(self):
        """Hard-block within the chassis disc, then a decaying penalty band outward.

        Vectorised dilation: a chamfer distance transform costs ~250k Python ops per control
        tick on an 80x80 window, which is slower than the 20 Hz loop it is supposed to serve.
        """
        blocked = self.cost >= 0.9
        if not blocked.any():
            return
        hard = int(round(CHASSIS_R / RES))
        soft = int(round((CHASSIS_R + INFLATE + 0.12) / RES))
        cur = blocked
        for k in range(1, soft + 1):
            nxt = _dilate(cur)
            ring = nxt & ~cur
            if not ring.any():
                break
            if k <= hard:
                self.cost[nxt] = 1.0
            else:
                self.cost[ring] = np.maximum(self.cost[ring], 0.85 - 0.12 * (k - hard))
            cur = nxt

    def at(self, wx, wy):
        c = int((wx - self.origin[0]) / RES)
        r = int((wy - self.origin[1]) / RES)
        if not (0 <= r < SIDE and 0 <= c < SIDE):
            return 1.0
        return float(self.cost[r, c])

    def free_at(self, wx, wy):
        return self.at(wx, wy) < COST_BLOCK


def _dilate(m):
    out = m.copy()
    out[1:, :] |= m[:-1, :]
    out[:-1, :] |= m[1:, :]
    out[:, 1:] |= m[:, :-1]
    out[:, :-1] |= m[:, 1:]
    out[1:, 1:] |= m[:-1, :-1]
    out[1:, :-1] |= m[:-1, 1:]
    out[:-1, 1:] |= m[1:, :-1]
    out[:-1, :-1] |= m[1:, 1:]
    return out

# -------------------------------------------------------------------- A* ----
def astar(cm, start, goal, expand=4000):
    """Cheapest 8-connected route on cost; returns world points or None."""
    def cell(p):
        return int((p[0] - cm.origin[0]) / RES), int((p[1] - cm.origin[1]) / RES)

    s, g = cell(start), cell(goal)
    if not (0 <= s[0] < SIDE and 0 <= s[1] < SIDE):
        return None
    if not (0 <= g[0] < SIDE and 0 <= g[1] < SIDE):
        return None
    openq = [(math.hypot(g[0] - s[0], g[1] - s[1]) * RES, 0.0, s)]
    came, cost = {s: None}, {s: 0.0}
    seen = 0
    while openq:
        _, gc, cur = heapq.heappop(openq)
        if gc > cost.get(cur, 1e9):
            continue
        seen += 1
        if seen > expand:
            return None
        if cur == g:
            break
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if not (dc or dr):
                    continue
                nxt = (cur[0] + dc, cur[1] + dr)
                if not (0 <= nxt[0] < SIDE and 0 <= nxt[1] < SIDE):
                    continue
                k = cm.cost[nxt[1], nxt[0]]
                if k >= COST_BLOCK:
                    continue
                step = math.hypot(dc, dr) * RES * (1.0 + 6.0 * k)
                ng = gc + step
                if ng < cost.get(nxt, 1e9):
                    cost[nxt] = ng
                    came[nxt] = cur
                    heapq.heappush(openq, (ng + math.hypot(g[0] - nxt[0], g[1] - nxt[1]) * RES,
                                           ng, nxt))
    if g not in came:
        return None
    path, n = [], g
    while n is not None:
        path.append((n[0] * RES + cm.origin[0] + RES / 2.0,
                     n[1] * RES + cm.origin[1] + RES / 2.0))
        n = came[n]
    path.reverse()
    return path


# ------------------------------------------------------------ trajectory ----
def _arc(x, y, yaw, v, w, horizon, dt=0.1):
    v, w = achieved(v, w)
    pts = []
    steps = int(round(horizon / dt))
    for _ in range(steps):
        yaw2 = yaw + w * dt
        x += v * dt * math.cos(yaw2)
        y += v * dt * math.sin(yaw2)
        yaw = yaw2
        pts.append((x, y, yaw))
    return pts


def _hits(cm, pts):
    for x, y, _ in pts:
        if cm.at(x, y) >= COST_BLOCK:
            return True
    return False


def _clearance(cm, pts):
    return min((1.0 - cm.at(x, y)) for x, y, _ in pts)


def steer(cm, pose, goal, allow_reverse=True):
    """Best (v, w) toward `goal`, or None when nothing in the window is safe.

    Progress is measured as the reduction of the distance to the goal at the end of each
    simulated arc, so an arc that swings wide around a corner beats one that runs straight
    into it -- and one that reverses out of a dead end beats neither only when it is the only
    option left, which is what the reverse penalty is for.
    """
    x, y, yaw = pose[0], pose[1], pose[2]
    d0 = math.hypot(goal[0] - x, goal[1] - y)
    best = None
    vs = [MAX_V, MAX_V * 0.5, 0.0] + ([-0.07] if allow_reverse else [])
    for v in vs:
        for i in range(13):
            w = -MAX_W + i * (2 * MAX_W / 12)
            horizon = 2.2 if v >= 0 else 1.4
            pts = _arc(x, y, yaw, v, w, horizon)
            if _hits(cm, pts):
                continue
            end = pts[-1]
            gain = d0 - math.hypot(goal[0] - end[0], goal[1] - end[1])
            align = math.cos(_wrap(math.atan2(goal[1] - end[1], goal[0] - end[0]) - end[2]))
            score = 2.2 * gain + 0.5 * align + 0.6 * _clearance(cm, pts)
            score -= 0.25 * abs(w) + (0.35 if v < 0 else 0.0)
            if v == 0.0 and w == 0.0:
                score -= 0.3
            if best is None or score > best[0]:
                best = (score, v, w, pts)
    if best is None:
        return None
    return best[1], best[2], best[0]


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def frontier_goal(cm, pose, reach=1.9, first=0.4):
    """Where to head next: the direction with the most still-unmapped ground, biased left.

    Following the edge of what has been mapped is what turns "explore" into "walk the room's
    outline" -- the boundary of the known region *is* the wall the lidar has not rounded yet.
    Sweeping k from negative to positive and keeping strict improvements means ties resolve to
    the left, so the tour runs counter-clockwise without a separate routing decision.
    """
    x, y, yaw = pose[0], pose[1], pose[2]
    best, best_score = None, 0
    for k in range(-6, 19):
        b = yaw + math.radians(10 * k)
        cb, sb = math.cos(b), math.sin(b)
        s, last, t = 0, None, first
        while t <= reach:
            px, py = x + t * cb, y + t * sb
            c = cm.at(px, py)
            if c >= COST_BLOCK:
                break
            last = (px, py)
            if abs(c - COST_UNKNOWN) < 0.03:
                s += 1
            t += 0.1
        if last and s > best_score:
            best_score, best = s, last
    return best if best_score else None


def global_frontier(world, pose, cap=30000):
    """Bearing toward the nearest unmeasured cell reachable through measured free space.

    Without this the tour dies the moment its immediate surroundings are mapped: the local
    frontier search sees nothing unknown within 1.9 m, and a car with nowhere to go just
    stands in the middle of a half-known room turning its head. The growing grid is 24 m
    across, so the real frontier is almost always in it -- it is simply outside the planning
    window, and only the *direction* has to come from the global view.
    """
    lo = world.lo
    n = world.n
    sc = int((pose[0] - world.ox) / world.res)
    sr = int((pose[1] - world.oy) / world.res)
    if not (0 <= sc < n and 0 <= sr < n) or lo[sr, sc] >= 0.65:
        return None
    # Walls here are one cell thick and the lidar cannot see straight into its own blind
    # wedge, so a single unmapped hole in a wall is normal. Without closing them the search
    # walks through the east wall and puts the frontier 1.4 m outside the room, which is the
    # dead end this car kept driving into.
    # Any positive evidence blocks, not just lo>=0.65: a wall cell that a neighbouring ray
    # clipped once or twice sits near zero and let the search leak north through the top wall,
    # which produced a frontier 1.2 m outside the room and a car creeping along the wall for
    # 130 s chasing it.
    blocked = _dilate(lo > 0.0)
    seen = np.zeros((n, n), bool)
    seen[sr, sc] = True
    dq = deque([(sr, sc)])
    count = 0
    while dq and count < cap:
        r, c = dq.popleft()
        count += 1
        # 4-connected on purpose: mapper draws walls as single-cell lines, and a diagonal
        # step slips between them. The first version put the frontier on the far side of the
        # east wall and steered the car into the corner, where it then sat for 100 s.
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            rr, cc = r + dr, c + dc
            if not (0 <= rr < n and 0 <= cc < n) or seen[rr, cc] or blocked[rr, cc]:
                continue
            v = lo[rr, cc]
            if v == 0.0:                       # never measured: that is the frontier
                tx = world.ox + (cc + 0.5) * world.res
                ty = world.oy + (rr + 0.5) * world.res
                return math.atan2(ty - pose[1], tx - pose[0])
            seen[rr, cc] = True
            dq.append((rr, cc))
    return None


def candidates(cm, world, pose, home, reach=1.6):
    """Every goal worth trying, best first: a frontier inside the planning window, the bearing
    of the nearest frontier anywhere in the growing map, then home."""
    x, y = pose[0], pose[1]
    out = []
    g = frontier_goal(cm, pose)
    if g is not None:
        out.append((g, "局部边界"))
    b = global_frontier(world, pose)
    if b is not None:
        out.append(((x + reach * math.cos(b), y + reach * math.sin(b)), "全局边界"))
    out.append((home, "已探明回家"))
    return out


def plan(cm, world, pose, home, reach=1.6, avoid=()):
    """The first goal A* can actually route to, else that goal with no path.

    Two different failures used to be answered by driving on. A goal behind a wall is not a
    failure, it is the same instruction pulled closer along its bearing -- handled by the clip
    loop. A goal that is unreachable at any distance is a real failure (the frontier search
    finds holes: the lidar measures straight through a thin wall and reports floor on the far
    side), and it used to fall through to steering at the raw goal, which is how the car
    pressed into the south-west corner for 170 s. Now it returns path=None and the caller
    unsticks instead of advancing.
    """
    cands = candidates(cm, world, pose, home, reach)
    for goal, src in cands:
        if any(math.hypot(goal[0] - ax, goal[1] - ay) < 0.6 for ax, ay in avoid):
            continue
        for d in (reach, 0.9, 0.5):
            g = _clip(pose, goal, d)
            p = astar(cm, (pose[0], pose[1]), g)
            if p:
                return g, p, src
    return cands[0][0], None, cands[0][1]


class Pilot:
    """One navigation decision per tick, shared by the simulator and the car.

    Keeping this loop in two places is how the behaviour that got validated in simulation and
    the behaviour that drove on the floor diverged once already.
    """

    FOLD = 0.2                 # s between lidar folds into the growing map
    REPLAN = 0.5               # s; A* at 20 Hz would starve the cmd_vel stream
    STALL_MOVE = 0.15          # m ...
    STALL_SECS = 8.0           # ... covered in this long, or the car is stuck
    FIRST_STALL = 12.0         # s before stall counting starts, the opening manoeuvre is slow
    RECOVER = 2.0              # s of reverse-turn per unstick
    AVOID_TTL = 25.0           # s a goal with no route stays blacklisted

    def __init__(self, world, home):
        self.world = world
        self.home = home
        self.avoid = []        # (x, y, t) goals that had no route
        self.last_fold = self.last_plan = None
        self.goal = self.path = None
        self.src = ""
        self.stalls = self.recovers = self.noreplan = 0
        self.pp = None
        self.recover_until = -1.0
        self.rec_bearing = 0.0
        self.cm = None

    def step(self, t, pose, pairs):
        """(v, w) to command at elapsed time t, given the pose and the live lidar ring."""
        x, y, yaw = pose[0], pose[1], pose[2]
        if self.last_fold is None or t - self.last_fold >= self.FOLD:
            self.world.integrate(pose, pairs)
            self.last_fold = t
        cm = Costmap(self.world.grid(), self.world.res, self.world.ox, self.world.oy)
        cm.rebuild(pose, pairs)
        self.cm = cm
        if self.last_plan is None or t - self.last_plan >= self.REPLAN:
            self.avoid = [a for a in self.avoid if t - a[2] < self.AVOID_TTL]
            self.goal, self.path, self.src = plan(
                cm, self.world, pose, self.home,
                avoid=[(a[0], a[1]) for a in self.avoid])
            self.last_plan = t
            if self.path is None:
                self.noreplan += 1
                self.avoid.append((self.goal[0], self.goal[1], t))
        if self.pp is None or math.hypot(x - self.pp[0], y - self.pp[1]) > self.STALL_MOVE:
            self.pp = (x, y, t)
        elif t > self.FIRST_STALL and t - self.pp[2] > self.STALL_SECS:
            self.pp = (x, y, t)
            self._unstick(t, cm, pose)
        elif self.path is None:
            self._unstick(t, cm, pose)
        if t < self.recover_until:
            err = (self.rec_bearing - math.degrees(yaw) + 180.0) % 360.0 - 180.0
            return -0.06, max(-0.5, min(0.5, math.radians(err)))
        if self.path is None:
            self.recovers += 1
            return -0.05, 0.35
        got = steer(cm, pose, along(self.path, 0.6))
        if got is None:
            self.recovers += 1
            return -0.05, 0.35
        return got[0], got[1]

    def _unstick(self, t, cm, pose):
        """Start a reverse-turn toward the widest gap, if one is not already running."""
        if t < self.recover_until:
            return
        self.stalls += 1
        self.recover_until = t + self.RECOVER
        self.rec_bearing = math.degrees(pose[2]) + open_bearing(cm, pose) + 180.0


def _clip(pose, g, reach):
    x, y = pose[0], pose[1]
    d = math.hypot(g[0] - x, g[1] - y)
    if d > reach:
        t = reach / d
        return (x + (g[0] - x) * t, y + (g[1] - y) * t)
    return g


def open_bearing(cm, pose, reach=1.4, bins=12):
    """Body-frame bearing with the most unobstructed ground, for getting out of a stall.

    A stalled car is usually against the inflation band of something the planner walked it
    into. Pointing at the widest gap and reversing that way is the only manoeuvre that works
    without knowing what it got wrong.
    """
    x, y, yaw = pose[0], pose[1], pose[2]
    best, best_score = 0.0, -1.0
    for i in range(bins):
        b = 2 * math.pi * i / bins
        cb, sb = math.cos(b), math.sin(b)
        s, t = 0.0, 0.25
        while t <= reach:
            c = cm.at(x + t * cb * math.cos(yaw) - t * sb * math.sin(yaw),
                      y + t * cb * math.sin(yaw) + t * sb * math.cos(yaw))
            if c >= COST_BLOCK:
                break
            s += 1.0 - c
            t += 0.1
        else:
            s += 2.0                       # nothing blocked within reach at all
        if s > best_score:
            best_score, best = s, math.degrees(b)
    return best


def along(path, d):
    """Point `d` metres along a polyline (clamped to its end).

    The scorer aims at this rather than at the raw goal: the goal can be 2 m away and behind a
    corner, and an arc judged against it looks good for 20 ticks and then collides.
    """
    if not path:
        return None
    acc = 0.0
    for a, b in zip(path, path[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if acc + L >= d:
            t = 0.0 if L == 0 else (d - acc) / L
            return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        acc += L
    return path[-1]


def save_world(world, out_dir, travelled=None):
    """Write grid.npy + map.json + map.png in the layout mapper.py's tools already read."""
    import json
    import os
    import struct
    import zlib
    os.makedirs(out_dir, exist_ok=True)
    lo = world.grid()
    np.save(os.path.join(out_dir, "grid.npy"), lo)
    json.dump({"resolution": world.res, "cells": world.n,
               "origin_x": world.ox, "origin_y": world.oy,
               "extent_m": world.n * world.res, "frames": world.frames,
               "free_cells": int((lo <= -2.0).sum()),
               "occupied_cells": int((lo >= 0.65).sum()),
               "pose": list(travelled[-1]) if travelled else None,
               "path": [list(p) for p in (travelled or [])]},
              open(os.path.join(out_dir, "map.json"), "w", encoding="utf-8"))

    img = np.full((world.n, world.n, 3), 120, np.uint8)
    img[lo <= 0.4] = (245, 245, 245)
    img[lo >= 0.65] = (30, 30, 30)
    img = np.flipud(img)
    for x, y in (travelled or []):
        c = int((x - world.ox) / world.res)
        r = world.n - 1 - int((y - world.oy) / world.res)
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if 0 <= r + dr < world.n and 0 <= c + dc < world.n:
                    img[r + dr, c + dc] = (220, 40, 40)

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))
    big = np.repeat(np.repeat(img, 2, 0), 2, 1)
    h, w, _ = big.shape
    raw = b"".join(b"\x00" + big[r].tobytes() for r in range(h))
    with open(os.path.join(out_dir, "map.png"), "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))
    return out_dir


# ------------------------------------------------------------- perimeter ----
class World:
    """A growing odom-frame occupancy grid, fed by the bridge's own filtered scans.

    Deliberately simpler than mapper.py: no per-bearing median, no IQR rejection, no still
    gate. Those make a *picture* worth looking at; this only has to answer "is there something
    there", and it has to be live, because the saved map stops being the truth the moment the
    car drives somewhere the map never covered.
    """

    def __init__(self, extent=24.0, res=RES, v_free=-0.35, v_occ=0.9):
        self.res = res
        self.n = int(extent / res)
        self.ox = -extent / 2.0
        self.oy = -extent / 2.0
        self.lo = np.zeros((self.n, self.n), np.float32)
        self.v_free = v_free
        self.v_occ = v_occ
        self.frames = 0

    def integrate(self, pose, pairs, every=3, samples=12):
        """pose = (x, y, yaw); pairs = body-frame (deg, range) already self/blind-masked."""
        if not pairs:
            return
        x, y, yaw = pose[0], pose[1], pose[2]
        sel = pairs[::every]
        th = np.radians(np.array([p[0] for p in sel])) + yaw
        rr = np.array([p[1] for p in sel])
        ca, sa = np.cos(th), np.sin(th)
        ts = np.linspace(0.08, 1.0, samples)              # skip the first 8 cm: chassis
        for t in ts:
            d = rr * t
            self._add(x + d * ca, y + d * sa, self.v_free)
        self._add(x + rr * ca, y + rr * sa, self.v_occ)
        self.frames += 1

    def _add(self, xs, ys, v):
        c = np.floor((xs - self.ox) / self.res).astype(np.int64)
        r = np.floor((ys - self.oy) / self.res).astype(np.int64)
        ok = (c >= 0) & (c < self.n) & (r >= 0) & (r < self.n)
        idx = (r[ok], c[ok])
        cur = self.lo[idx] + v
        np.clip(cur, -2.5, 2.5, out=cur)
        self.lo[idx] = cur

    def grid(self):
        return self.lo


# ------------------------------------------------------ scan-to-map match ----
MIN_WALLS = 120            # wall cells below which the map cannot answer "where am I"
MATCH_WALL = 0.4           # log-odds a cell must reach to count as a surface for matching
MIN_SUPPORT = 25           # endpoints that must fall on measured ground for a score to mean anything
SIGMA = 0.10               # m: how far a return may sit from a mapped wall and still count
DCAP = 8                   # cells of distance beyond which a return contributes nothing
# m: how far in front of a measured endpoint a crossed wall cell still counts as a contradiction.
# Set by lead sweep on the live map (_veto_probe.py), not by taste: walls here are drawn 2-3 cells
# thick, so the old two-cell guard let a ray "see through" the far edge of its own wall and the
# metric sat at 0.5 even at a pose the matcher rates 0.995 -- saturated input, no usable contrast.
THROUGH_LEAD = 0.30


def _dist_cap(mask, cap):
    """Distance to the nearest True cell, saturated at cap+1, by repeated dilation.

    A capped distance is all the matcher needs, and computing it this way keeps it to `cap`
    vectorised dilations instead of the per-cell Python loop _chamfer does.
    """
    reached = mask.copy()
    d = np.where(mask, 0, cap + 1).astype(np.int16)
    for k in range(1, cap + 1):
        reached = _dilate(reached)
        d[reached & (d > k)] = k
    return d


def scan_offset(world, pose, pairs, reach=1.6, step=0.10, every=6, rival_sep=0.5):
    """Score the live sweep against the map at every translation within `reach`, and report
    the winner alongside the score the sweep already has where the odometry claims we are.

    Translation only, deliberately: the gyro feeds odom's yaw, so a carry that also turns the
    car is captured by odometry, and searching rotation as well would fit a degree of freedom
    the sensors already measure -- while making the grid search 10x more expensive.

    The score is a distance falloff, not a boolean "is there a wall nearby". The boolean
    version was measured and thrown out: on a real dense map it read 97.5% agreement for a
    sweep shifted a full metre off the map, because a two-cell dilation covers most of a small
    room. Sharpness is the entire point of the metric.

    Returns None when the map is too thin to have an opinion.
    """
    lo = world.grid()
    # 0.4, not the renderer's 0.65: a wall cell that got one occupancy vote and one free pass
    # through it sits at 0.55, and calling that "not a wall" starves the matcher on a young map.
    occ = lo >= MATCH_WALL
    walls = int(occ.sum())
    if walls < MIN_WALLS:
        return None
    n = lo.shape[0]
    dist = _dist_cap(occ, DCAP)
    lut = np.exp(-(np.arange(DCAP + 2) * world.res) ** 2 / (2.0 * SIGMA ** 2))
    seen_d = _dilate(lo != 0.0)

    x, y, yaw = pose[0], pose[1], pose[2]
    sel = list(pairs)[::every]
    if len(sel) < MIN_SUPPORT * 2:
        return None
    th = np.radians(np.array([p[0] for p in sel])) + yaw
    rr = np.array([p[1] for p in sel])
    cx = (x + rr * np.cos(th) - world.ox) / world.res
    cy = (y + rr * np.sin(th) - world.oy) / world.res

    dm = np.arange(-reach, reach + 1e-9, step)      # candidate offsets, metres
    dc = np.rint(dm / world.res).astype(np.int64)   # ... in cells
    m = len(dm)
    z = m // 2                                      # dm is symmetric, so index m//2 is zero
    # gx varies with the x offset, gy with the y one; broadcasting them against each other
    # gives the whole (dy, dx, point) search volume in one fancy index -- 33x33x111 stays ms.
    gx = np.rint(cx[None, :] + dc[:, None]).astype(np.int64)     # (m, pts)
    gy = np.rint(cy[None, :] + dc[:, None]).astype(np.int64)     # (m, pts)
    okx = (gx >= 0) & (gx < n)
    oky = (gy >= 0) & (gy < n)
    inside = oky[:, None, :] & okx[None, :, :]
    ix = np.clip(gx, 0, n - 1)[None, :, :]
    iy = np.clip(gy, 0, n - 1)[:, None, :]
    w = lut[dist[iy, ix]]
    sup = seen_d[iy, ix] & inside
    nsup = sup.sum(2)
    score = np.where(nsup > 0, (w * sup).sum(2) / np.maximum(nsup, 1), 0.0)
    score[nsup < MIN_SUPPORT] = 0.0

    i = int(np.argmax(score))
    r, c = i // m, i % m                            # rows are dy, columns are dx
    # The runner-up: the best score anywhere at least `rival_sep` from the winner. Two parallel
    # walls in a rectangular room both explain the same sweep, and a matcher that cannot say
    # "and nothing else fits" is not localising -- it is guessing. Reporting it is what lets the
    # caller refuse to move the pose on an ambiguous map.
    sep = np.hypot(dm[None, :] - dm[c], dm[:, None] - dm[r])
    alt = np.where(sep >= rival_sep, score, 0.0)
    j = int(np.argmax(alt))
    jr, jc = j // m, j % m
    # How much of this sweep lands on ground the map has never measured. Without it, "nothing
    # matches" has two completely different causes that score identically: the pose (or the map)
    # is wrong, and the car has simply driven somewhere nobody has looked yet. A caller that
    # cannot tell those apart aborts exploration the moment exploration starts working.
    inb = okx[z] & oky[z]
    onmap = seen_d[np.clip(gy[z], 0, n - 1), np.clip(gx[z], 0, n - 1)] & inb
    outside = float(1.0 - onmap.mean())
    # Sub-cell refinement: the grid search lands on a 5 cm staircase, and drift we are asked to
    # suppress is 2-3 cm -- a matcher that can only answer in 5 cm steps turns every small error
    # into quantisation noise and reports that as "drift". A parabola through the peak and its two
    # neighbours costs three lookups and resolves where the peak actually sits.
    def refine(idx, line):
        if 0 < idx < m - 1:
            a0, a1, a2 = line[idx - 1], line[idx], line[idx + 1]
            den = a0 - 2.0 * a1 + a2
            if den < -1e-9:                       # curved downward: a real peak, not a ridge
                return float(max(-step, min(step, 0.5 * (a0 - a2) / den * step)))
        return 0.0
    return {"dx": float(dm[c]), "dy": float(dm[r]),
            "fdx": float(dm[c]) + refine(c, score[r]), "fdy": float(dm[r]) + refine(r, score[:, c]),
            "best": float(score[r, c]), "zero": float(score[z, z]),
            "rival": float(score[jr, jc]), "rdx": float(dm[jc]), "rdy": float(dm[jr]),
            "support": int(nsup[r, c]), "walls": walls, "points": len(sel),
            "outside": outside}


def through_wall(world, pose, pairs, every=6, reach=6.0, start=0.10, lead=None):
    """What share of the sweep sees *past* something the map claims is in the way.

    `scan_offset` only asks the lidar's question -- does each return land near a mapped surface
    -- and that question is one-sided in a way that bites: a patch of phantom walls lowers the
    score at every pose, and the offset search then "wins" by sliding the sweep *onto* the
    phantom mass. This asks the map's question instead. March each ray out from the candidate
    pose to the distance the lidar actually measured and count the rays that cross an occupied
    cell on the way. A real carry moves the whole sweep together, so the recovered pose scores
    about nothing here; a wrongly-drawn or stale patch contradicts the sweep wherever you stand,
    so no candidate beats the pose odometry claims by much. Measured on the live car: the
    matcher's preferred (+-1.5 m) placement contradicted 80% of the sweep against 47% where the
    odometry said the car was, which is what "you were carried" should have had to prove and did
    not.

    `lead` is how far in front of the measured endpoint a crossed cell still counts. It is the
    whole design: the old guard was two cells (0.10 m), and this map draws its walls 2-3 cells
    thick, so a ray landing on the far side of its own wall "looked through" it. At a pose the
    matcher itself rates zero=best=0.995 the share was still 0.506 -- the metric had no headroom
    left, and a veto built on it could only ever excuse a carry (measured 2026-10-18: a real
    +0.80 m jump came back as "地图画错了"). Crossing something *well before* the surface you
    measured is the contradiction worth counting; crossing the band you are standing in front of
    is a drawing artifact.

    Returns None when the sweep is too thin to have an opinion.
    """
    if lead is None:
        lead = THROUGH_LEAD
    lo = world.grid()
    occ = lo >= MATCH_WALL
    n = lo.shape[0]
    x, y, yaw = pose[0], pose[1], pose[2]
    sel = list(pairs)[::every]
    if len(sel) < MIN_SUPPORT:
        return None
    th = np.radians(np.array([p[0] for p in sel])) + yaw
    rr = np.array([p[1] for p in sel])
    todo = np.ones(len(sel), bool)
    cross = np.zeros(len(sel), bool)
    for t in np.arange(start, reach, world.res):
        if not todo.any():
            break
        c = np.floor((x + t * np.cos(th) - world.ox) / world.res).astype(np.int64)
        r = np.floor((y + t * np.sin(th) - world.oy) / world.res).astype(np.int64)
        inb = (c >= 0) & (c < n) & (r >= 0) & (r < n)
        hit = np.zeros(len(sel), bool)
        hit[inb] = occ[r[inb], c[inb]]
        cross |= todo & hit & (t < rr - lead)             # a wall at the endpoint is a match
        todo &= ~hit
    return float(cross.mean())


def frontier_view(world, pose, cap=40000, bins=12):
    """What the map still does not know, and in which direction -- the global view the
    chassis has always had internally and no caller outside the Pilot could ask for.

    Walks the *measured* ground from the car's cell the same way `global_frontier` does (same
    dilation to seal hairline holes, same 4-connectivity so a single-cell wall stays a wall),
    but does not stop at the first unknown cell: it counts them, keeps the nearest, and bins
    the rest by bearing. "Nearest frontier" answers where to step next; "which sector holds
    most of the unknown" is what a high-level planner needs to decide whether another lap is
    worth driving at all -- `global_frontier` returning None only says no frontier is
    *reachable*, which is a different claim from the room being finished.
    """
    lo = world.lo
    n = world.n
    sc = int((pose[0] - world.ox) / world.res)
    sr = int((pose[1] - world.oy) / world.res)
    if not (0 <= sc < n and 0 <= sr < n):
        return None
    blocked = _dilate(lo > 0.0)
    seen = np.zeros((n, n), bool)
    if blocked[sr, sc]:                    # the car's own cell reads as wall: the pose is the lie
        return {"frontier": 0, "nearest": None, "bearing": None, "sectors": [0] * bins,
                "walked": 0, "in_wall": True}
    seen[sr, sc] = True
    dq = deque([(sr, sc)])
    walked = 0
    hits = []                              # (distance, map-frame bearing) of every unknown reached
    while dq and walked < cap:
        r, c = dq.popleft()
        walked += 1
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            rr, cc = r + dr, c + dc
            if not (0 <= rr < n and 0 <= cc < n) or seen[rr, cc]:
                continue
            if lo[rr, cc] == 0.0:          # never measured: count it, do not walk into it
                seen[rr, cc] = True
                tx = world.ox + (cc + 0.5) * world.res - pose[0]
                ty = world.oy + (rr + 0.5) * world.res - pose[1]
                hits.append((math.hypot(tx, ty), math.atan2(ty, tx)))
            elif not blocked[rr, cc]:
                seen[rr, cc] = True
                dq.append((rr, cc))
    if not hits:
        return {"frontier": 0, "nearest": None, "bearing": None, "sectors": [0] * bins,
                "walked": walked, "in_wall": False}
    hits.sort()
    # Body-frame bearing, positive = left: the convention this whole stack settled on
    # (SCAN_SIGN=1.0, odom yaw is CCW+). A map-frame bearing would make the caller look up the
    # heading before it could execute the answer.
    rel = _wrap(hits[0][1] - pose[2])
    sectors = [0] * bins
    for _d, b in hits:
        sectors[int((_wrap(b - pose[2]) + math.pi) / (2 * math.pi) * bins) % bins] += 1
    return {"frontier": len(hits), "nearest": round(hits[0][0], 2),
            "bearing": round(math.degrees(rel), 1), "sectors": sectors,
            "walked": walked, "in_wall": False}


def load_grid(path):
    """grid.npy plus its map.json header."""
    import json
    import os
    lo = np.load(os.path.join(path, "grid.npy"))
    meta = json.load(open(os.path.join(path, "map.json"), encoding="utf-8"))
    return lo, meta["resolution"], meta["origin_x"], meta["origin_y"]


def perimeter_waypoints(lo, res, ox, oy, standoff=0.30, samples=24, step=0.9):
    """A ring of goals around the free space, ordered counter-clockwise.

    Radial march from the free-space centroid to the edge of the drivable region in `samples`
    bearings. This is deliberately not a connected path -- A* connects it, one leg at a time,
    against the live costmap. The ring only has to say "roughly there, then roughly there".
    """
    n = lo.shape[0]
    free = lo <= 0.4
    if free.sum() < 40:
        return []
    rr, cc = np.nonzero(free)
    cx = ox + (cc.mean() + 0.5) * res
    cy = oy + (rr.mean() + 0.5) * res
    stand = int(round(standoff / res))
    dist = _chamfer(lo >= 0.65)
    driv = free & (dist >= stand)
    pts = []
    for i in range(samples):
        b = 2 * math.pi * i / samples
        best = None
        t = stand * res
        while t < 0.45 * n * res:
            px, py = cx + t * math.cos(b), cy + t * math.sin(b)
            c = int((px - ox) / res)
            r = int((py - oy) / res)
            if not (0 <= r < n and 0 <= c < n) or not driv[r, c]:
                break
            best = (px, py)
            t += res / 2.0
        if best:
            pts.append(best)
    if len(pts) < 4:
        return pts
    # thin to roughly `step` metres apart so a leg is one A* call, not twenty
    out, last = [], None
    for p in pts:
        if last is None or math.hypot(p[0] - last[0], p[1] - last[1]) >= step:
            out.append(p)
            last = p
    if len(out) > 2 and math.hypot(out[-1][0] - out[0][0], out[-1][1] - out[0][1]) < step:
        out.pop()
    return out


def _chamfer(occ):
    """Cell distance to the nearest occupied cell (multi-source, 8-connected)."""
    n = occ.shape[0]
    big = 1e3
    d = np.where(occ, 0.0, big).astype(np.float32)
    for r in range(n):
        for c in range(n):
            v = d[r, c]
            if v == 0.0:
                continue
            if c and d[r, c - 1] + 1 < v:
                v = d[r, c - 1] + 1
            if r and d[r - 1, c] + 1 < v:
                v = d[r - 1, c] + 1
            if r and c and d[r - 1, c - 1] + 1.414 < v:
                v = d[r - 1, c - 1] + 1.414
            d[r, c] = v
    for r in range(n - 1, -1, -1):
        for c in range(n - 1, -1, -1):
            v = d[r, c]
            if c + 1 < n and d[r, c + 1] + 1 < v:
                v = d[r, c + 1] + 1
            if r + 1 < n and d[r + 1, c] + 1 < v:
                v = d[r + 1, c] + 1
            if r + 1 < n and c + 1 < n and d[r + 1, c + 1] + 1.414 < v:
                v = d[r + 1, c + 1] + 1.414
            d[r, c] = v
    return d
