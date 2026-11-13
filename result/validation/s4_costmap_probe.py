import sys, time, math, rclpy, subprocess
sys.path.insert(0, 'src/mrs_navigation/scripts')
from nav_check import spawn_box
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init()
n = rclpy.create_node('probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
grids = {}
def mk(name):
    def cb(m): grids[name] = m
    return cb
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(OccupancyGrid, 'global_costmap/costmap', mk('global'), tl)
n.create_subscription(OccupancyGrid, 'local_costmap/costmap', mk('local'), 10)
n.create_subscription(OccupancyGrid, 'map', mk('map'), tl)
scan = {}
n.create_subscription(LaserScan, 'scan', lambda m: scan.__setitem__('s', m), QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
def show(tag):
    t0 = time.time()
    while time.time() - t0 < 1.5: rclpy.spin_once(n, timeout_sec=0.1)
    s = scan['s']; i0 = len(s.ranges)//2 if s.angle_min < -3 else 0
    front = [r for k, r in enumerate(s.ranges) if abs(s.angle_min + k*s.angle_increment) < 0.15]
    print(f'{tag}: scan front min {min(front):.2f} m')
    for name in ('map', 'global', 'local'):
        g = grids.get(name)
        if g is None: print('  no', name); continue
        # box at map (0.5, 0.0) -> print row y=0 from x=0.2..0.9
        vals = []
        for mx in [0.2 + 0.05*k for k in range(15)]:
            cx = int((mx - g.info.origin.position.x)/g.info.resolution); cy = int((0.0 - g.info.origin.position.y)/g.info.resolution)
            vals.append(g.data[cy*g.info.width + cx] if 0 <= cx < g.info.width and 0 <= cy < g.info.height else -9)
        print(f'  {name:6s} frame={g.header.frame_id} x=0.20..0.90:', vals)
time.sleep(1)
show('before')
print('spawn', spawn_box('floor', 'probe_box', 1.0, 0.5))
for k in range(4):
    show(f'after {k*1.5+1.5:.1f}s')
