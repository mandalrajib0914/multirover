"""Offline: render the victim texture into a level camera at known poses; check world centre."""
import sys, math
import numpy as np, cv2
sys.path.insert(0, 'src/mrs_perception')
from mrs_perception.aruco_victim_node import make_detector, marker_poses, victim_centre
tex = cv2.imread('src/mrs_gazebo/models/victim/materials/textures/aruco_0.png', cv2.IMREAD_GRAYSCALE)
fx = 320 / math.tan(1.089 / 2); K = np.array([[fx, 0, 320], [0, fx, 240], [0, 0, 1.0]]); D = np.zeros(5)
det = make_detector('DICT_4X4_50')
# world: victim centre at (3.0, 4.75, 0.10), face normal -y; marker centre at (3.0, 4.648, 0.10)
C = np.array([3.0, 4.75, 0.10]); nface = np.array([0, -1.0, 0]); M = C + nface * 0.102
# optical frame of a level camera looking along world direction d: z=d, x=right, y=down
def cam_R(d):
    z = d / np.linalg.norm(d); x = np.cross(z, [0, 0, 1.0]); x /= np.linalg.norm(x); y = np.cross(z, x)
    return np.stack([x, y, z], 1)   # columns: optical axes in world = camera->world rotation
worst = 0
for rng in (0.8, 1.5, 2.0, 2.5):
    for bearing in (0, 20, 35, 50):          # camera position angle off the face normal
        b = math.radians(bearing)
        cam = M + rng * np.array([math.sin(b), -math.cos(b), 0]); cam[2] = 0.075   # camera height
        Rw = cam_R(M - cam)
        # face corners in world (0.2 m texture), project
        h = 0.1; right = np.array([1.0, 0, 0]); up = np.array([0, 0, 1.0])  # seen from -y: +x is right? face normal -y -> viewer at -y sees +x on the left
        # camera looks along +y with +z up, so +x is its right: not mirrored
        corners_w = [M - right * h + up * h, M + right * h + up * h, M + right * h - up * h, M - right * h - up * h]
        pc = np.array([Rw.T @ (p - cam) for p in corners_w]); ip = (K @ pc.T).T; ip = ip[:, :2] / ip[:, 2:]
        H = cv2.getPerspectiveTransform(np.array([[0, 0], [511, 0], [511, 511], [0, 511]], np.float32), ip.astype(np.float32))
        img = cv2.warpPerspective(tex, H, (640, 480), borderValue=90)
        c, ids = det(img)
        if ids is None:
            print(f'range {rng} bearing {bearing:2d}: not detected'); continue
        poses = marker_poses(c[0], 0.10, K, D)
        w, nh = victim_centre(poses, Rw, cam, 0.10)
        err = np.linalg.norm(w[:2] - C[:2]); worst = max(worst, err)
        print(f'range {rng} bearing {bearing:2d}: centre ({w[0]:.3f}, {w[1]:.3f}) err {err*100:4.1f} cm, normal {nh[:2].round(2)}')
print(f'worst xy error {worst*100:.1f} cm')
