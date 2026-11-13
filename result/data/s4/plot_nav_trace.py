# Plot a nav_check.py --trace CSV over the floor plan (S4).
#   python3 result/data/s4/plot_nav_trace.py result/data/s4/nav_check_trace.csv out.png [x0 x1 y0 y1]
import csv, math, sys, yaml
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
plan = yaml.safe_load(open('src/mrs_gazebo/config/floor_plan.yaml'))
FP = [(0.09, 0.12), (0.09, -0.12), (-0.14, -0.12), (-0.14, 0.12)]
rows = list(csv.DictReader(open(sys.argv[1])))
fig, ax = plt.subplots(figsize=(12, 10))
for x1, y1, x2, y2 in plan['walls']:
    ax.plot([x1, x2], [y1, y2], 'k-', lw=3)
for cx, cy, sx, sy in plan['obstacles']:
    ax.add_patch(Rectangle((cx - sx/2, cy - sy/2), sx, sy, color='gray'))
cols = {'goal': 'tab:blue', 'obstacle': 'tab:green', 'off_map': 'tab:purple', 'in_obstacle': 'tab:red'}
tests = {}
for r in rows:
    if r['test'].startswith('box:'):
        ax.add_patch(Rectangle((float(r['x']) - .15, float(r['y']) - .15), .3, .3, color='orange'))
        continue
    tests.setdefault(r['test'], []).append((float(r['x']), float(r['y']), float(r['yaw'])))
for name, pts in tests.items():
    ax.plot([p[0] for p in pts], [p[1] for p in pts], '-', color=cols.get(name, 'k'), label=name, lw=1)
    for i in range(0, len(pts), max(1, len(pts) // 25)):
        x, y, t = pts[i]; c, s = math.cos(t), math.sin(t)
        ax.add_patch(Polygon([(x + c*px - s*py, y + s*px + c*py) for px, py in FP], fill=False, ec=cols.get(name, 'k'), lw=0.5))
ax.set_aspect('equal'); ax.legend(); ax.grid(alpha=.3)
xl = [float(v) for v in sys.argv[3:5]] if len(sys.argv) > 4 else None
if xl: ax.set_xlim(xl); ax.set_ylim([float(v) for v in sys.argv[5:7]])
fig.savefig(sys.argv[2], dpi=90, bbox_inches='tight')
