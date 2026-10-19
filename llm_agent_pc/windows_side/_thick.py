"""Wall-thickness (离空距离) of the *console's* saved map: _edge_test.py needs mapper's map.json.

A real wall is a one-cell line, so every occupied cell sits next to a free one and the distance to
the nearest non-occupied cell is 1. Odometry error while turning paints the same face twice a
fraction of a metre apart and the band thickens. Mean above ~1.3 = doubled edges, not new detail.

    _rosrun.bat _thick.py [cache_console/grid.npy]
"""
import sys

import numpy as np


def thickness(occ):
    """Chebyshev distance to the nearest non-occupied cell, by repeated erosion."""
    d = occ.copy()
    level = np.zeros(occ.shape, np.int32)
    lev = 0
    while d.any():
        lev += 1
        p = np.pad(d, 1)
        core = np.ones_like(d)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                core &= p[1 + dy:1 + dy + d.shape[0], 1 + dx:1 + dx + d.shape[1]]
        level[d & ~core] = lev
        d = core
    return level


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "cache_console/grid.npy"
    a = np.load(path)
    occ = a >= 0.65
    if not occ.any():
        print("%s: 没有占据格" % path)
        return
    lev = thickness(occ)[occ]
    print("%s: %d 格，离空 mean %.3f  p50 %d  p90 %d  max %d，>2 格占 %.1f%%"
          % (path, occ.sum(), lev.mean(), np.percentile(lev, 50), np.percentile(lev, 90),
             lev.max(), 100.0 * (lev > 2).mean()))
    print("判据：单格线 mean≈1.0；>1.3 就是同面画了两遍（转弯糊图）。")


if __name__ == "__main__":
    main()
