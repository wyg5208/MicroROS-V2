import numpy as np, sys
src = r'C:\microros-agent\pc_side\cache_console\grid.npy'
g = np.load(src)
res = 0.05
sx = int(round(float(sys.argv[1]) / res))
sy = int(round(float(sys.argv[2]) / res))
out = np.zeros_like(g)
out[sy:, sx:] = g[:-sy, :-sx]
tmp = src + '.tmp'
with open(tmp, 'wb') as fh:
    np.save(fh, out)
import os
os.replace(tmp, src)
print('shifted by (%.2f, %.2f) m  occ %d -> %d' % (sx*res, sy*res, (g>=0.4).sum(), (out>=0.4).sum()))
