import math, sys, glob
import numpy as np
sys.path.insert(0, 'src/mrs_exploration')
sys.path.insert(0, 'src/mrs_bringup/scripts')
from simple_map_merge import merge_grids
from mrs_exploration.exploration_monitor import load_map_yaml
from nav_msgs.msg import OccupancyGrid
def to_msg(grid, res, ox, oy, yaw=0.0):
    m = OccupancyGrid(); m.info.resolution = res; m.info.height, m.info.width = grid.shape
    m.info.origin.position.x, m.info.origin.position.y = ox, oy
    m.data = np.where(grid == 1, 100, np.where(grid == 0, 0, -1)).astype(np.int8).ravel().tolist(); return m
gt, gres, (gox, goy) = load_map_yaml('src/mrs_gazebo/maps/floor_gt.yaml')
f = sorted(glob.glob('data/s5/*run2/map.yaml'))[0]
sm, sres, (sox, soy) = load_map_yaml(f)
# 1) one robot, spawn pose -> compare with gt like map_vs_gt (coverage)
out, x0, y0 = merge_grids({'r1': to_msg(sm, sres, sox, soy)}, {'r1': (0.5, 0.5, 0.0)}, 0.05)
rows, cols = np.indices(gt.shape)
wx, wy = gox + (cols + .5) * gres, goy + (rows + .5) * gres
i = np.floor((wy - y0) / 0.05).astype(int); j = np.floor((wx - x0) / 0.05).astype(int)
ok = (i >= 0) & (i < out.shape[0]) & (j >= 0) & (j < out.shape[1])
v = np.full(gt.shape, -1); v[ok] = out[i[ok], j[ok]]
print(f'test1 coverage via merge: {((gt == 0) & (v == 0)).sum() / (gt == 0).sum() * 100:.1f}%  (map_vs_gt said 95.4%)')
print(f'       wall cells that are gt walls within 1 cell: ', end='')
occ = v == 100
from scipy.ndimage import binary_dilation
print(f'{(occ & binary_dilation(gt == 1, iterations=2)).sum() / occ.sum() * 100:.1f}%')
# 2) same map rotated: express robot map in a frame rotated by 90 deg at a different spot;
#    merged result must be the same world picture
yaw = math.pi / 2
# a frame F with pose (px,py,yaw) in world; the map's cells in F: rotate content by -yaw.
# easiest check: synthetic L-shaped grid
g = np.full((40, 60), -1, np.int8); g[5:35, 5:55] = 0; g[5, 5:55] = 1; g[5:35, 5] = 1
m = to_msg(g, 0.05, -1.0125, -0.5125)
a, ax, ay = merge_grids({'a': m}, {'a': (2.0, 1.0, 0.0)}, 0.05)
b, bx, by = merge_grids({'b': m}, {'b': (2.0, 1.0, yaw)}, 0.05)
# rotate b back: world point p -> rotate about (2,1) by -yaw should land where a has it
pts_a = np.argwhere(a == 100); pts_b = np.argwhere(b == 100)
def world(ix, x0, y0): return np.stack([x0 + (ix[:, 1] + .5) * .05, y0 + (ix[:, 0] + .5) * .05], 1)
wa, wb = world(pts_a, ax, ay), world(pts_b, bx, by)
c, s = math.cos(-yaw), math.sin(-yaw)
wb_back = np.stack([2 + c * (wb[:, 0] - 2) - s * (wb[:, 1] - 1), 1 + s * (wb[:, 0] - 2) + c * (wb[:, 1] - 1)], 1)
d = np.min(np.linalg.norm(wb_back[:, None] - wa[None], axis=2), axis=1)
print(f'test2 rotated frame: occupied cells a={len(wa)} b={len(wb)}, max misplacement {d.max():.3f} m (<= 1 cell = 0.05 ok)')
# 3) occupied beats free: two robots, one sees free where the other sees a wall
g1 = np.zeros((20, 20), np.int8); g2 = np.full((20, 20), -1, np.int8); g2[10, :] = 1
o, _, _ = merge_grids({'a': to_msg(g1, .05, .0125, .0125), "b": to_msg(g2, .05, .0125, .0125)}, {'a': (0, 0, 0), 'b': (0, 0, 0)}, .05)
print('test3 occupied wins:', (o == 100).sum() == 20, ' free elsewhere:', (o == 0).sum() == 380, ' no unknown:', (o == -1).sum())
