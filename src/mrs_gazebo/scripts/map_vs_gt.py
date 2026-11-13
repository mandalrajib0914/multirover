#!/usr/bin/env python3
"""Compare a saved SLAM map with the ground-truth map from make_floor.py.

  ros2 run mrs_gazebo map_vs_gt.py src/mrs_navigation/maps/floor_v1.yaml [--robot robot1|world]
      [--gt <share>/maps/floor_gt.yaml] [--png overlay.png] [--tol 0.10]

The SLAM map frame is the robot's start pose, so it is moved into the world frame with the
robot's spawn pose from floor_plan.yaml. Then, on the ground-truth grid:
  wall precision  share of SLAM occupied cells within --tol of a true wall
  wall recall     share of true wall cells within --tol of a SLAM occupied cell
  coverage        share of true free cells that SLAM marked free
  wall F1         2PR / (P + R) of the two above
  free IoU        |SLAM free AND true free| / |SLAM free OR true free|
The overlay PNG shows true walls in black, SLAM walls in red (orange where they miss) and
SLAM free space in light blue.
"""
import argparse
import math
import os

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import yaml


def load_map(path):
    """Return (grid, res, origin_xy); grid: 1 occupied, 0 free, -1 unknown, row 0 = min y."""
    with open(path) as f:
        meta = yaml.safe_load(f)
    img = cv2.imread(os.path.join(os.path.dirname(path), meta['image']), cv2.IMREAD_GRAYSCALE)
    p = (255 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
    grid = np.full(img.shape, -1, np.int8)
    grid[p > meta['occupied_thresh']] = 1
    grid[p < meta['free_thresh']] = 0
    return np.flipud(grid), meta['resolution'], meta['origin'][:2]


def main():
    gz_pkg = get_package_share_directory('mrs_gazebo')
    ap = argparse.ArgumentParser()
    ap.add_argument('slam_map')
    ap.add_argument('--gt', default=os.path.join(gz_pkg, 'maps', 'floor_gt.yaml'))
    ap.add_argument('--plan', default=os.path.join(gz_pkg, 'config', 'floor_plan.yaml'))
    ap.add_argument('--robot', default='robot1',
                    help="whose spawn pose is the map origin; 'world' for a world-frame map")
    ap.add_argument('--tol', type=float, default=0.10, help='wall match tolerance, m')
    ap.add_argument('--victim', type=float, nargs=2, metavar=('X', 'Y'), default=None,
                    help='victim box position if not the floor_plan one (S8 trials): the truth '
                         'is then floor_gt_novictim plus a box here')
    ap.add_argument('--victim-size', type=float, default=0.2)
    ap.add_argument('--png', default=None, help='overlay image (default: next to the map)')
    args = ap.parse_args()

    if args.robot == 'world':   # already in the world frame, e.g. the merged /map (S6)
        sx, sy, syaw = 0.0, 0.0, 0.0
    else:
        with open(args.plan) as f:
            sx, sy, syaw = yaml.safe_load(f)['spawn_poses'][args.robot]
    if args.victim is not None:
        gt_file = os.path.join(os.path.dirname(args.gt), 'floor_gt_novictim.yaml')
        gt, gres, (gox, goy) = load_map(gt_file)
        vx, vy, vs = args.victim[0], args.victim[1], args.victim_size
        # same rounding as make_floor.py; row 0 = min y
        a0, a1 = int(round((vx - vs / 2 - gox) / gres)), int(round((vx + vs / 2 - gox) / gres))
        b0, b1 = int(round((vy - vs / 2 - goy) / gres)), int(round((vy + vs / 2 - goy) / gres))
        gt[b0:b1, a0:a1] = 1
    else:
        gt, gres, (gox, goy) = load_map(args.gt)
    sm, sres, (sox, soy) = load_map(args.slam_map)

    # For every GT cell centre: world -> SLAM map frame -> SLAM cell (nearest neighbour).
    rows, cols = np.indices(gt.shape)
    wx, wy = gox + (cols + 0.5) * gres - sx, goy + (rows + 0.5) * gres - sy
    c, s = math.cos(-syaw), math.sin(-syaw)
    mx, my = c * wx - s * wy, s * wx + c * wy
    si, sj = np.floor((my - soy) / sres).astype(int), np.floor((mx - sox) / sres).astype(int)
    inside = (si >= 0) & (si < sm.shape[0]) & (sj >= 0) & (sj < sm.shape[1])
    slam = np.full(gt.shape, -1, np.int8)
    slam[inside] = sm[si[inside], sj[inside]]

    k = 2 * int(round(args.tol / gres)) + 1
    kernel = np.ones((k, k), np.uint8)
    gt_occ, sl_occ = (gt == 1).astype(np.uint8), (slam == 1).astype(np.uint8)
    near_gt, near_sl = cv2.dilate(gt_occ, kernel) > 0, cv2.dilate(sl_occ, kernel) > 0
    precision = (sl_occ.astype(bool) & near_gt).sum() / max(sl_occ.sum(), 1)
    recall = (gt_occ.astype(bool) & near_sl).sum() / max(gt_occ.sum(), 1)
    coverage = ((gt == 0) & (slam == 0)).sum() / max((gt == 0).sum(), 1)
    f1 = 2 * precision * recall / max(precision + recall, 1e-9)
    free_iou = ((gt == 0) & (slam == 0)).sum() / max(((gt == 0) | (slam == 0)).sum(), 1)
    print(f'wall precision {precision * 100:5.1f}%   (SLAM walls within {args.tol:.2f} m of a true wall)')
    print(f'wall recall    {recall * 100:5.1f}%   (true walls seen by SLAM)')
    print(f'wall F1        {f1 * 100:5.1f}%   (harmonic mean of the two)')
    print(f'free IoU       {free_iou * 100:5.1f}%   (free cells: SLAM and truth, over either)')
    print(f'coverage       {coverage * 100:5.1f}%   (true free space mapped free)')

    img = np.full(gt.shape + (3,), 255, np.uint8)
    img[slam == 0] = (250, 225, 190)
    img[gt == 1] = (0, 0, 0)
    img[sl_occ.astype(bool) & near_gt] = (40, 40, 220)
    img[sl_occ.astype(bool) & ~near_gt] = (0, 140, 255)
    img = cv2.resize(np.flipud(img), None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
    out = args.png or os.path.splitext(args.slam_map)[0] + '_vs_gt.png'
    cv2.imwrite(out, img)
    print(f'overlay: {out}')


if __name__ == '__main__':
    main()
