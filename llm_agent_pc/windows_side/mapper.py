"""Occupancy-grid mapper for the MicroROS-V2 car: /scan + /odom_raw -> cache/map.png.

Self-written because this Windows ROS2 install has no slam_toolbox / cartographer / nav2.
The gimbal hides ~44 degrees straight ahead, so that slice is left UNKNOWN (never free);
it fills in by itself as the car turns and drives.

Run with the ROS2 environment sourced (see _rosrun.bat). Only numpy + stdlib.
"""
import argparse
import json
import math
import os
import struct
import threading
import time
import zlib

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from car_mcp_server import BLIND_WEDGE, SCAN_SIGN, SELF_RANGE_MAX, wrap180

OCC_LO, FREE_LO = 2.0, -0.6      # log-odds thresholds; 2.0 needs three frames of evidence
BIN_DEG = 1.0                    # bearing bin used for the temporal filter


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Grid:
    """Log-odds occupancy grid.

    Every integrated scan votes: a ray's cells go negative, its endpoint goes positive,
    both clamped. Redundant scans therefore saturate real surfaces instead of smearing
    the map.
    """

    def __init__(self, extent, res, p_free=0.38, p_occ=0.72):
        self.res = res
        self.n = int(extent / res)
        self.ox = -extent / 2.0
        self.oy = -extent / 2.0
        self.lo = np.zeros((self.n, self.n), dtype=np.float32)
        self.v_free = math.log(p_free / (1.0 - p_free))
        self.v_occ = math.log(p_occ / (1.0 - p_occ))

    def _idx(self, xs, ys):
        c = np.floor((xs - self.ox) / self.res).astype(np.int64)
        r = np.floor((ys - self.oy) / self.res).astype(np.int64)
        ok = (c >= 0) & (c < self.n) & (r >= 0) & (r < self.n)
        key = np.unique(r[ok] * self.n + c[ok])   # one vote per cell per scan
        return key // self.n, key % self.n

    def mark_free(self, xs, ys):
        r, c = self._idx(xs, ys)
        if r.size:
            cur = self.lo[r, c] + self.v_free          # fancy indexing copies, so write back
            np.maximum(cur, -2.5, out=cur)
            self.lo[r, c] = cur

    def mark_occ(self, xs, ys):
        r, c = self._idx(xs, ys)
        if r.size:
            cur = self.lo[r, c] + self.v_occ
            np.minimum(cur, 3.5, out=cur)
            self.lo[r, c] = cur

    def counts(self):
        occ = int(np.count_nonzero(denoise(self.lo)))
        free = int(np.count_nonzero(self.lo <= FREE_LO))
        return free, occ


class Mapper(Node):
    """Folds scans into the grid, but never straight from one frame.

    A single frame of this T-MINI contains floor-graze and multipath outliers, and the
    per-bearing median over the last several frames is tight (IQR < 5 cm on real faces).
    So rays are binned by bearing, and a scan is integrated only once the car has moved
    enough or enough time passed: the median per bin is the ray's range, and bins whose
    spread stays large are dropped instead of guessed, which leaves them UNKNOWN.
    """

    def __init__(self, args):
        super().__init__("pc_mapper")
        self.args = args
        self.lock = threading.Lock()
        self.buf = {}           # bearing bin -> list of range samples since the last flush
        self.seen = 0
        self.pose = None        # (x, y, yaw, monotonic)
        self.pose_acc = []      # poses sampled since the last fold
        self.grid = Grid(args.extent, args.resolution)
        self.last_int = None    # pose the previous integrated scan used
        self.last_int_t = 0.0
        self.dropped = 0        # bins rejected for spread this flush
        self.moving_dropped = 0  # whole windows rejected for not being still
        self.path = []
        self.frames = 0
        self.create_subscription(LaserScan, "/scan", self._scan_cb, self._qos())
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, self._qos())

    @staticmethod
    def _qos():
        # micro-ROS endpoints are BEST_EFFORT; a RELIABLE subscription matches nothing
        return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

    def _scan_cb(self, msg):
        amin = math.degrees(msg.angle_min)
        ainc = math.degrees(msg.angle_increment)
        step = max(1, int(self.args.ray_step))
        rng = np.asarray(msg.ranges, dtype=np.float64)
        with self.lock:
            for i in range(0, len(rng), step):
                r = rng[i]
                if not math.isfinite(r) or r <= SELF_RANGE_MAX or r >= msg.range_max:
                    continue
                a = SCAN_SIGN * (amin + i * ainc)
                if BLIND_WEDGE[0] <= wrap180(a) <= BLIND_WEDGE[1]:
                    continue
                b = int((a + 360.0) % 360.0 / BIN_DEG)
                v = self.buf.setdefault(b, [])
                v.append(r)
                if len(v) > self.args.buffer:
                    del v[0]
            self.seen += 1

    def _odom_cb(self, msg):
        p = msg.pose.pose
        y = yaw_of(p.orientation)
        with self.lock:
            self.pose = (p.position.x, p.position.y, y, time.monotonic())
            self.pose_acc.append((p.position.x, p.position.y, math.sin(y), math.cos(y)))

    def _mean_pose(self):
        """Pose of the window the buffered rays were measured in, plus how much it moved.

        Folding the rays collected across a 4-degree turn with the pose at the END of
        that turn puts the whole sweep several centimetres off at wall range, which is
        what doubles edges during a rotation. The circular mean is the pose the samples
        actually belong to.

        The spreads matter separately: this board's wheel odometry holds yaw while
        parked and while turning, but drifts it a few degrees per 0.4 m of translation,
        so a window with motion in it cannot be trusted even after mean-pose folding.
        """
        with self.lock:
            acc, self.pose_acc = self.pose_acc, []
        if not acc:
            return (self.pose[:3] + (0.0, 0.0)) if self.pose else None
        a = np.asarray(acc, dtype=np.float64)
        mx, my = float(a[:, 0].mean()), float(a[:, 1].mean())
        yaw = float(math.atan2(a[:, 2].mean(), a[:, 3].mean()))
        pspread = float(np.hypot(a[:, 0] - mx, a[:, 1] - my).max())
        d = np.degrees(np.arctan2(a[:, 2], a[:, 3])) - math.degrees(yaw)
        yspread = float(np.radians(np.abs((d + 180.0) % 360.0 - 180.0).max()))
        return (mx, my, yaw, pspread, yspread)

    def _filtered_scan(self):
        """Median range per bearing bin; reject bins whose spread stays large.

        A jittery ray still proves everything before its nearest return is free, so it is
        kept as a free-space ray with no occupancy vote. That fills the spokes a dropped
        bin would otherwise leave in the map.
        """
        with self.lock:
            buf, self.buf = self.buf, {}
            n_seen, self.seen = self.seen, 0
        if n_seen == 0:
            return None
        all_a, all_r, occ_a, occ_r = [], [], [], []
        dropped = 0
        for b, v in buf.items():
            a = np.asarray(v)
            deg = (b + 0.5) * BIN_DEG
            if len(v) < self.args.min_samples:
                # too few echoes to place a surface, but the ray clearly got this far
                all_a.append(deg)
                all_r.append(float(a.min()))
                continue
            spread = float(np.percentile(a, 75) - np.percentile(a, 25))
            if spread > self.args.spread_max:
                dropped += 1
                all_a.append(deg)
                all_r.append(float(a.min()))
                continue
            med = float(np.median(a))
            all_a.append(deg)
            all_r.append(med)
            occ_a.append(deg)
            occ_r.append(med)
        # a direction with no echo at all stays UNKNOWN: an absorbing surface (dark cloth)
        # returns nothing at 2 m just as happily as an open corridor returns nothing at 8 m
        return (np.asarray(all_a), np.asarray(all_r),
                np.asarray(occ_a), np.asarray(occ_r), dropped)

    def integrate_once(self):
        """Fold the accumulated scans into the grid. Returns False when nothing usable."""
        with self.lock:
            pose, has_buf = self.pose, bool(self.buf)
        if pose is None or not has_buf:
            return False
        if time.monotonic() - pose[3] > self.args.max_odom_age:
            return False
        px, py, yaw = pose[0], pose[1], pose[2]
        if self.last_int is not None:
            # a parked car must still build evidence, but folding every 15 Hz frame while
            # driving doubles the smear, so gate on pose or on a slow timer
            moved = math.hypot(px - self.last_int[0], py - self.last_int[1])
            turned = abs((yaw - self.last_int[2] + math.pi) % (2 * math.pi) - math.pi)
            waiting = time.monotonic() - self.last_int_t < self.args.update_seconds
            if moved < self.args.update_dist and turned < math.radians(self.args.update_angle) \
                    and waiting:
                return False
        scan = self._filtered_scan()
        if scan is None or scan[2].size == 0:
            return False
        all_a, all_r, occ_a, occ_r, dropped = scan
        mp = self._mean_pose()
        if mp is None:
            return False
        mx, my, myaw, pspread, yspread = mp
        # Stop-and-go is the only way to use this odometry for translation: a window with
        # motion in it carries a heading error that grows with distance travelled, and at
        # wall range 5 degrees is 17 cm of lateral smear -- exactly the doubled edges a
        # continuous drive produces. Rays measured while rolling are discarded outright.
        still = self.args.still_max > 0 and self.args.still_angle > 0
        if still and (pspread > self.args.still_max
                      or yspread > math.radians(self.args.still_angle)):
            self.moving_dropped += 1
            return False
        self.last_int = (px, py, yaw)
        self.last_int_t = time.monotonic()
        self.dropped = dropped
        # bearings are degrees, yaw is radians: convert before projecting
        th = np.radians(all_a) + myaw
        cos_a, sin_a = np.cos(th), np.sin(th)
        # walk each ray outward one cell per step, stopping at that ray's own return
        steps = int(self.args.raycast_max / self.args.resolution)
        for k in range(1, steps + 1):
            t = k * self.args.resolution
            live = all_r > t
            if not live.any():
                break
            self.grid.mark_free(mx + t * cos_a[live], my + t * sin_a[live])
        tho = np.radians(occ_a) + myaw
        # one 1-degree bin is about one cell wide at these ranges, so a single cell per
        # bin leaves pinholes that the neighbour filter then prunes; thicken along the ray
        for t in (0.0, -self.args.resolution):
            rr = np.maximum(occ_r + t, 0.0)
            self.grid.mark_occ(mx + rr * np.cos(tho), my + rr * np.sin(tho))
        self.frames += 1
        if not self.path or math.hypot(px - self.path[-1][0],
                                       py - self.path[-1][1]) > 0.05:
            self.path.append((px, py))
        return True


def write_png(path, rgb):
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[i].tobytes() for i in range(h))

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


def denoise(lo, passes=1):
    """Keep only occupied cells that have occupied neighbours.

    The T-MINI still throws occasional multipath returns off fabric, which leaves
    single-cell salt. Real surfaces come in runs along a line. One pass, not two: a wall
    seen from one pose is a single-cell-thick arc, and a second pass ate 40%% of them.
    """
    m = lo >= OCC_LO
    for _ in range(passes):
        p = np.pad(m, 1)
        n = np.zeros(m.shape, dtype=np.int8)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr or dc:
                    n += p[1 + dr:1 + dr + m.shape[0], 1 + dc:1 + dc + m.shape[1]]
        m = m & (n >= 2)
    return m


def _rotate(img, deg):
    """Turn an image counter-clockwise about its centre, bilinear, grey outside.

    Bilinear rather than nearest: the walls in this map are exactly one cell thick, and a
    nearest-neighbour turn shatters them into dashes.
    """
    h, w = img.shape[:2]
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    rows, cols = np.mgrid[0:h, 0:w].astype(np.float64)
    du = cols - cx
    dv = (h - 1 - rows) - cy                       # image rows grow downward, math y up
    su = ca * du + sa * dv                         # src = R(-deg) * dst
    sv = -sa * du + ca * dv
    sc = su + cx
    sr = (h - 1) - (sv + cy)
    inside = (sc >= 0) & (sc <= w - 1) & (sr >= 0) & (sr <= h - 1)
    sc = np.clip(sc, 0, w - 1.001)
    sr = np.clip(sr, 0, h - 1.001)
    c0 = np.floor(sc).astype(int)
    r0 = np.floor(sr).astype(int)
    fc = (sc - c0)[..., None]
    fr = (sr - r0)[..., None]
    g = img.astype(np.float32)
    out = (g[r0, c0] * (1 - fr) * (1 - fc) + g[r0, c0 + 1] * (1 - fr) * fc
           + g[r0 + 1, c0] * fr * (1 - fc) + g[r0 + 1, c0 + 1] * fr * fc)
    out[~inside] = 205.0
    return np.rint(out).astype(np.uint8)


def _line(rgb, x0, y0, x1, y1, colour):
    h, w = rgb.shape[:2]
    n = int(max(abs(x1 - x0), abs(y1 - y0), 1))
    for i in range(n + 1):
        x = int(round(x0 + (x1 - x0) * i / n))
        y = int(round(y0 + (y1 - y0) * i / n))
        for dy in (-1, 0, 1):                      # 3 px wide, so it survives a thumbnail
            for dx in (-1, 0, 1):
                if 0 <= y + dy < h and 0 <= x + dx < w:
                    rgb[y + dy, x + dx] = colour


def render(node, scale=3, car_up=False):
    lo = node.grid.lo
    h, w = lo.shape
    t = np.clip((lo - FREE_LO) / (OCC_LO - FREE_LO), 0.0, 1.0)
    g = np.rint(250.0 - 225.0 * t).astype(np.uint8)
    g[lo == 0.0] = 205                             # never measured
    g[denoise(lo)] = 25
    rgb = np.dstack((g, g, g)).astype(np.uint8)
    for x, y in node.path:                         # travelled path
        c = int((x - node.grid.ox) / node.grid.res)
        r = int((y - node.grid.oy) / node.grid.res)
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                if 0 <= r + dr < h and 0 <= c + dc < w:
                    rgb[r + dr, c + dc] = (200, 60, 60)
    rgb = np.flipud(rgb)                           # +y up in the image
    yaw = math.degrees(node.pose[2]) if node.pose else 0.0
    deg = 90.0 - yaw if car_up else 0.0            # put the car's heading at the top
    if deg:
        rgb = _rotate(rgb, deg)
    rgb = np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)
    if node.pose:                                  # heading arrow, always: a map with no
        c = (node.pose[0] - node.grid.ox) / node.grid.res   # "which way is the car" mark
        r = (node.pose[1] - node.grid.oy) / node.grid.res   # reads as rotated the wrong way
        du = c - (w - 1) / 2.0
        dv = ((h - 1) - r) - (h - 1) / 2.0         # math y (up) of the flipped image
        ca, sa = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        ex = (ca * du - sa * dv + (w - 1) / 2.0) * scale
        ey = ((h - 1) - (sa * du + ca * dv + (h - 1) / 2.0)) * scale
        ha = math.radians(yaw + deg)               # heading in the finished image
        _line(rgb, int(round(ex)), int(round(ey)),
              int(round(ex + 26 * math.cos(ha))), int(round(ey - 26 * math.sin(ha))),
              (255, 255, 0))
    return rgb


def save(node, out_dir):
    png = os.path.join(out_dir, "map.png")
    write_png(png, render(node, car_up=node.args.car_up))
    # raw log-odds, so the map can be checked numerically instead of by eye
    np.save(os.path.join(out_dir, "grid.npy"), node.grid.lo)
    free, occ = node.grid.counts()
    with open(os.path.join(out_dir, "map.json"), "w", encoding="utf-8") as f:
        json.dump({"resolution": node.grid.res, "cells": node.grid.n,
                   "origin_x": node.grid.ox, "origin_y": node.grid.oy,
                   "extent_m": node.args.extent, "frames": node.frames,
                   "free_cells": free, "occupied_cells": occ,
                   "pose": list(node.pose[:3]) if node.pose else None,
                   "path": [[round(x, 3), round(y, 3)] for x, y in node.path]},
                  f, ensure_ascii=False, indent=1)
    return png, free, occ


def main():
    p = argparse.ArgumentParser(description="MicroROS-V2 occupancy grid mapper")
    p.add_argument("--ros-domain-id", type=int, default=40)
    p.add_argument("--odom-topic", default="/odom_raw")
    p.add_argument("--resolution", type=float, default=0.08,
                   help="cell size; 0.08 keeps neighbouring 0.54-degree rays merged")
    p.add_argument("--extent", type=float, default=12.0, help="square map edge in metres")
    p.add_argument("--ray-step", type=int, default=1, help="use every Nth lidar ray")
    p.add_argument("--min-samples", type=int, default=4,
                   help="bearing bins needing this many frames before they count")
    p.add_argument("--spread-max", type=float, default=0.12,
                   help="drop a bearing bin when its interquartile range exceeds this")
    p.add_argument("--still-max", type=float, default=0.02,
                   help="reject a window whose poses spread further than this; 0 disables")
    p.add_argument("--still-angle", type=float, default=0.8,
                   help="...or rotate more than this many degrees; 0 disables")
    p.add_argument("--buffer", type=int, default=40, help="range samples kept per bin")
    p.add_argument("--update-dist", type=float, default=0.05,
                   help="only integrate a scan after the car moved this far")
    p.add_argument("--update-angle", type=float, default=4.0,
                   help="...or turned this many degrees")
    p.add_argument("--update-seconds", type=float, default=1.0,
                   help="...or this much time passed, so a parked car still maps")
    p.add_argument("--raycast-max", type=float, default=6.0)
    p.add_argument("--max-odom-age", type=float, default=0.4)
    p.add_argument("--out-dir", default="cache")
    p.add_argument("--car-up", action="store_true",
                   help="turn the picture so the car's nose points up; without it the map "
                        "is drawn in odometry axes and looks rotated by the car's heading")
    p.add_argument("--load", default="",
                   help="directory holding a previous grid.npy, to keep drawing on top of "
                        "it; only valid while the car has not been moved by hand, since "
                        "this board cannot tell a lifted car from a driven one")
    p.add_argument("--duration", type=float, default=0.0, help="0 = run until Ctrl-C")
    p.add_argument("--interval", type=float, default=2.0, help="seconds between saves")
    args = p.parse_args()

    os.environ.setdefault("ROS_DOMAIN_ID", str(args.ros_domain_id))
    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    node = Mapper(args)
    if args.load:
        src = os.path.join(args.load, "grid.npy")
        prev = np.load(src)
        if prev.shape != node.grid.lo.shape:
            p.error("%s is %s but this run builds %s -- keep --extent and --resolution "
                    "identical, or the old cells would land in the wrong place"
                    % (src, prev.shape, node.grid.lo.shape))
        node.grid.lo = prev
        print("接着画：%s（已有 %d 个占据格）" % (src, int((prev >= OCC_LO).sum())))
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    print("建图中：/scan + %s -> %s/map.png（Ctrl-C 结束）" % (args.odom_topic, args.out_dir))
    t_start, t_save, skipped = time.monotonic(), 0.0, 0
    try:
        while rclpy.ok():
            time.sleep(0.1)
            if not node.integrate_once():
                skipped += 1
            now = time.monotonic()
            if now - t_save >= args.interval:
                t_save = now
                png, free, occ = save(node, args.out_dir)
                print("  %.0f 秒 折叠 %d 次 空格 %d 障碍 %d 弃用档 %d 移动窗 %d 位姿 (%.2f, %.2f) yaw %.0f°" % (
                    now - t_start, node.frames, free, occ, node.dropped, node.moving_dropped,
                    node.pose[0] if node.pose else 0, node.pose[1] if node.pose else 0,
                    math.degrees(node.pose[2]) if node.pose else 0))
                skipped = 0
                if args.duration and now - t_start > args.duration:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        png, free, occ = save(node, args.out_dir)
        rclpy.shutdown()
        print("已保存 %s（折叠 %d 次，空格 %d，障碍 %d）" % (png, node.frames, free, occ))


if __name__ == "__main__":
    main()
