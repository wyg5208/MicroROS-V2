"""Crop a saved occupancy map around a pose and draw the believed heading.

Usage: _zoom_map.py <map_dir> <out.png> <x> <y> <yaw_deg> [half_m]

The full grid is 24 m across, which renders as a speck of ink in the middle of a grey field;
this cuts a window around the car so the wall geometry is actually readable.
"""
import json
import math
import os
import struct
import sys
import zlib

import numpy as np

OCC_LO, FREE_LO = 0.65, 0.4
UNK, FREE, OCC = (120, 120, 120), (245, 245, 245), (25, 25, 25)


def png(path, rgb):
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[r].tobytes() for r in range(h))

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        f.write(chunk(b"IEND", b""))


def main():
    d, out = sys.argv[1], sys.argv[2]
    x, y, yaw = float(sys.argv[3]), float(sys.argv[4]), math.radians(float(sys.argv[5]))
    half = float(sys.argv[6]) if len(sys.argv) > 6 else 4.0

    meta = json.load(open(os.path.join(d, "map.json"), encoding="utf-8"))
    res, n = meta["resolution"], meta["cells"]
    ox, oy = meta["origin_x"], meta["origin_y"]
    lo = np.load(os.path.join(d, "grid.npy"))

    k = int(half / res) * 2 + 1
    c0 = int((x - ox) / res)
    r0 = int((y - oy) / res)
    cs = slice(max(0, c0 - k // 2), min(n, c0 + k // 2 + 1))
    rs = slice(max(0, r0 - k // 2), min(n, r0 + k // 2 + 1))
    sub = lo[rs, rs]
    img = np.zeros(sub.shape + (3,), np.uint8)
    for mask, col in ((sub > FREE_LO, UNK), (sub <= FREE_LO, FREE), (sub >= OCC_LO, OCC)):
        img[mask] = col
    img = np.flipud(img)

    def px(wx, wy):
        # the sub-array was flipud'd, so image row 0 is the highest grid row in the window
        return (int((wx - ox) / res) - cs.start,
                (rs.stop - rs.start - 1) - (int((wy - oy) / res) - rs.start))

    cc, rr = px(x, y)
    for dc in range(-2, 3):
        for dr in range(-2, 3):
            if 0 <= rr + dr < img.shape[0] and 0 <= cc + dc < img.shape[1]:
                img[rr + dr, cc + dc] = (220, 40, 40)
    t = res
    while t < 1.0:
        hx, hy = px(x + t * math.cos(yaw), y + t * math.sin(yaw))
        if 0 <= hy < img.shape[0] and 0 <= hx < img.shape[1]:
            img[hy, hx] = (255, 210, 0)
        t += res / 3.0
    up = np.repeat(np.repeat(img, 3, axis=0), 3, axis=1)
    png(out, up)
    print("窗口 %.1f×%.1f m，格 %dx%d，已写出 %s"
          % ((cs.stop - cs.start) * res, (rs.stop - rs.start) * res,
             *img.shape[:2], out))
    print("占据 %d 格，空闲 %d 格（窗口内）"
          % (int((sub >= OCC_LO).sum()), int((sub <= -2.0).sum())))


main()
