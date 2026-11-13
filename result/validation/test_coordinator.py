import glob, math, sys, time
import numpy as np
sys.path.insert(0, 'src/mrs_exploration'); sys.path.insert(0, 'src/mrs_bringup/scripts')
from mrs_exploration.coordinator import find_frontiers, path_costs, euclidean_costs, assign
from mrs_exploration.exploration_monitor import load_map_yaml
# A) path cost on an empty 4x4 m room with a wall across the middle and a 0.8 m door
res = 0.05; g = np.zeros((80, 80), np.int8); g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
g[:, 40] = 100; g[36:52, 40] = 0          # wall x=2.0, door y 1.8..2.6
o = (0.0, 0.0)
c = path_costs(g, res, o, [(1.0, 1.0)], [(3.0, 1.0), (1.0, 3.0)], 0.15)
print(f'A path (1,1)->(3,1) through door: {c[0,0]:.2f} m (straight 2.00; analytic via the door edge with 0.15 m clearance {2*math.hypot(1.0, 0.95):.2f})')
print(f'  path (1,1)->(1,3) same room: {c[0,1]:.2f} m (2.0 expected)')
g2 = g.copy(); g2[36:52, 40] = 100
print('  no door -> inf:', np.isinf(path_costs(g2, res, o, [(1, 1)], [(3, 1)], 0.15))[0, 0])
# B) frontiers: known left room, unknown right room -> one frontier at the door
gf = np.full((80, 80), -1, np.int8); gf[:, :41] = g[:, :41]; gf[36:52, 40] = 0
fr = find_frontiers(gf, res, o, 0.5, 0.2)
print('B frontiers:', [(round(f['goal'][0], 2), round(f['goal'][1], 2), round(f['gain'], 2)) for f in fr])
# C) assignment: 2 robots side by side, frontiers: one big near both, one small far
gains = np.array([3.0, 1.0]); goals = [(1.0, 0.0), (1.4, 0.0)]
cost = np.array([[1.0, 1.4], [1.0, 1.4]])
for st in ('nearest', 'greedy', 'hungarian'):
    print(f'C {st:9s}', assign(st, gains, cost, 0.5, 1.5, goals))
# D) real S5 map (partially explored copy): frontiers + timing
yml = sorted(glob.glob('results/s5/*run2/map.yaml'))[0]
sm, sres, so = load_map_yaml(yml)
part = sm.copy(); part[:, part.shape[1] // 2:] = -1          # hide the right half
occ = np.where(part == 1, 100, np.where(part == 0, 0, -1)).astype(np.int8)
t = time.time(); fr = find_frontiers(occ, sres, so, 0.5, 0.2); t1 = time.time() - t
t = time.time(); pc = path_costs(occ, sres, so, [(0.0, 0.0), (0.0, 0.5), (0.0, 1.0)], [f['goal'] for f in fr], 0.15); t2 = time.time() - t
print(f'D {len(fr)} frontiers in {t1*1000:.0f} ms, 3xF path costs in {t2*1000:.0f} ms; finite: {np.isfinite(pc).all()}')
print('  goals:', [(round(f['goal'][0], 2), round(f['goal'][1], 2), round(f['gain'], 2)) for f in fr])
full = np.where(sm == 1, 100, np.where(sm == 0, 0, -1)).astype(np.int8)
print('  complete map frontiers:', [(round(f['goal'][0], 2), round(f['goal'][1], 2), round(f['gain'], 2)) for f in find_frontiers(full, sres, so, 0.5, 0.2)])
