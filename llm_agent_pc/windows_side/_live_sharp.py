# -*- coding: utf-8 -*-
"""Sharpness of a live console map: usage _live_sharp.py <grid.npy> [<grid.npy> ...]

Same measure _edge_test.py uses for mapper output, applied to car_web's saved world grid so
two tours of the same room can be compared without the matcher's own opinion in the loop: a
pose that wanders re-draws each face twice and the occupied band thickens.
"""
import sys

import numpy as np

RES = 0.05
MAX_D = 8
NB = [(dy, dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if (dy, dx) != (0, 0)]


def bands(m):
    """Chebyshev distance from every occupied cell to the nearest open one (same as _edge_test).
    Copied rather than imported: _edge_test runs its report at import time and its helper needs
    mapper, which needs the ROS environment -- this script has to work against plain python."""
    d = np.full(m.shape, MAX_D + 1, dtype=np.int16)
    d[~m] = 0
    for k in range(1, MAX_D + 1):
        shell = d == k - 1
        near = np.zeros(m.shape, dtype=bool)
        for dy, dx in NB:
            near |= np.roll(np.roll(shell, -dy, axis=0), -dx, axis=1)
        d[m & (d > k) & near] = k
    return d



def report(path):
    g = np.load(path)
    m = g >= 0.65
    m[:, :1] = m[:, -1:] = m[:1, :] = m[-1:, :] = False   # np.roll wraps; keep the frame out
    t = bands(m)[m]
    hist = np.bincount(t, minlength=9)
    print("%-28s 障碍格 %5d  平均离空 %.3f  中位 %d  最厚 %d  占比 %.4f"
          % (path, int(m.sum()), t.mean(), int(np.median(t)), int(t.max()), m.mean()))
    print("                              离空分布 " + "  ".join(
        "%d格:%d" % (k, hist[k]) for k in range(1, 6)) + "   (%.2f m/cell)" % RES)
    return t.mean()


if __name__ == "__main__":
    for p in sys.argv[1:]:
        report(p)
