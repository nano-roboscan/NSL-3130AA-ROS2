#!/usr/bin/env python3
"""
Extrinsic calibration: LiDAR point cloud ↔ Camera image.
Refactored from sierrabase_driver/src/extrinsic/extrinsic_calib_node.py.

Changes from original:
  - --camera-id instead of --instance-name
  - Intrinsic source priority:
      a. --camera-info-yaml (explicit path)
      b. ~/calib_output/{camera_id}_intrinsic.yml  (OpenCV FileStorage)
      c. ~/.ros/camera_info/camera_{camera_id}.yaml (ROS format)
      d. Live /camera/rgb/camera_info topic
  - Output: ~/calib_output/{ID}_extrinsic.yml (OpenCV FileStorage, R 3x3 + t 3x1)
  - Also saves {ID}_camera_to_lidar.csv (3x4 [R|t]) for backward compatibility
  - Dataset/corner files in ~/calib_output/extrinsic_{camera_id}/

Workflow:
  1. Launch alongside camera + lidar drivers.
  2. Terminal prompts: s=store frame, c=calibrate, r=reset, q=quit.
  3. [s] Opens two matplotlib windows:
       - Image window: click 4 matching corners on the calibration target.
       - 3D window:    click 4 corresponding LiDAR points.
  4. Repeat [s] for 5+ frames from different viewpoints.
  5. [c] Runs solvePnPRansac → saves R/t YML + CSV + camera intrinsics YAML.
"""

import argparse
import multiprocessing
import os
import sys
import threading
from pathlib import Path

import cv2
import matplotlib
import numpy as np
import yaml

import rclpy
import message_filters
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2_util

matplotlib.use('TkAgg')

# ── ROI for 3D point selection (metres) ──────────────────────────────────────
ROI = dict(x_min=-2.0, x_max=2.0, y_min=-1.0, y_max=1.0, z_min=0.0, z_max=3.0)

CV_BRIDGE = CvBridge()
_g_pause  = False
_g_store  = False
_g_calib  = False
_g_reset  = False
_g_lock   = threading.Lock()


# ── Camera intrinsics helpers ─────────────────────────────────────────────────

def _make_cam_params(K, D, P, dist_model, W, H):
    model = 'fisheye' if dist_model in ('equidistant', 'fisheye') else 'pinhole'
    return dict(camera_model=model, distortion_model=dist_model,
                camera_matrix=K, dist_coeffs=D, projection_matrix=P,
                image_width=W, image_height=H)


def _load_intrinsics_cv_yml(path: str) -> dict:
    """Load intrinsics from OpenCV FileStorage .yml (written by intrinsic_calibration_node.py)."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError(f'Cannot open OpenCV FileStorage: {path}')
    K = fs.getNode('camera_matrix').mat()
    D = fs.getNode('distortion_coefficients').mat()
    dm_node = fs.getNode('distortion_model')
    dm = dm_node.string() if not dm_node.empty() else 'plumb_bob'
    W_node = fs.getNode('image_width')
    H_node = fs.getNode('image_height')
    W = int(W_node.real()) if not W_node.empty() else 0
    H = int(H_node.real()) if not H_node.empty() else 0
    fs.release()
    if K is None or K.size == 0:
        raise ValueError(f'camera_matrix empty in {path}')
    if D is None or D.size == 0:
        D = np.zeros((1, 5), np.float64)
    P = np.zeros((3, 4), np.float64)
    P[:3, :3] = K
    return _make_cam_params(K, D.reshape(-1, 1), P, dm or 'plumb_bob', W, H)


def _load_intrinsics_yaml(path: str) -> dict:
    """Load intrinsics from ROS camera_info YAML or custom flat YAML."""
    with open(path) as f:
        d = yaml.safe_load(f)

    if 'camera_matrix' in d:                              # ROS cameracalibrator format
        K = np.array(d['camera_matrix']['data'],          np.float64).reshape(3, 3)
        D = np.array(d['distortion_coefficients']['data'], np.float64).reshape(-1, 1)
        dist_model = d.get('distortion_model', 'plumb_bob')
        W, H = int(d.get('image_width', 0)), int(d.get('image_height', 0))
        P_data = d.get('projection_matrix', {}).get('data', [K[0, 0], 0, K[0, 2], 0,
                                                               0, K[1, 1], K[1, 2], 0,
                                                               0, 0, 1, 0])
        P = np.array(P_data, np.float64).reshape(3, 4)
    elif 'K' in d:                                        # custom flat format
        K = np.array(d['K'], np.float64).reshape(3, 3)
        D = np.array(d['D'], np.float64).reshape(-1, 1)
        dist_model = d.get('distortion_model', 'plumb_bob')
        W, H = int(d.get('image_width', 0)), int(d.get('image_height', 0))
        P = np.zeros((3, 4), np.float64)
        P[:3, :3] = K
    else:
        raise ValueError(f'Unrecognised camera YAML format: {path}')

    return _make_cam_params(K, D, P, dist_model, W, H)


def _cam_params_from_info_msg(msg: CameraInfo) -> dict:
    K = np.array(msg.k, np.float64).reshape(3, 3)
    D = np.array(msg.d, np.float64).reshape(-1, 1)
    if D.size == 0:
        D = np.zeros((5, 1), np.float64)
    P = np.array(msg.p, np.float64).reshape(3, 4)
    model = (msg.distortion_model or '').strip().lower()
    return _make_cam_params(K, D, P, model, int(msg.width), int(msg.height))


# ── PnP helpers ───────────────────────────────────────────────────────────────

def _solve_pnp(cam_params, corners_3d, corners_2d):
    obj = np.ascontiguousarray(corners_3d.reshape(-1, 1, 3), np.float64)
    K   = cam_params['camera_matrix']
    D   = cam_params['dist_coeffs']

    if cam_params['camera_model'] == 'fisheye':
        img_pts = cv2.fisheye.undistortPoints(
            np.ascontiguousarray(corners_2d.reshape(-1, 1, 2), np.float64),
            K, D, P=np.eye(3, np.float64))
        solve_K, solve_D = np.eye(3, np.float64), None
    else:
        img_pts   = np.ascontiguousarray(corners_2d.reshape(-1, 1, 2), np.float64)
        solve_K, solve_D = K, D

    ok, r, t, inliers = cv2.solvePnPRansac(
        obj, img_pts, solve_K, solve_D, flags=cv2.SOLVEPNP_ITERATIVE)
    if ok and inliers is not None and len(inliers) >= 3:
        idx = inliers.reshape(-1)
        r, t = cv2.solvePnPRefineLM(obj[idx], img_pts[idx], solve_K, solve_D, r, t)
    return ok, r, t, inliers


def _project_points(cam_params, corners_3d, r, t):
    K, D = cam_params['camera_matrix'], cam_params['dist_coeffs']
    if cam_params['camera_model'] == 'fisheye':
        pts, _ = cv2.fisheye.projectPoints(
            corners_3d.reshape(-1, 1, 3).astype(np.float64), r, t, K, D)
    else:
        pts, _ = cv2.projectPoints(corners_3d.astype(np.float64), r, t, K, D)
    return pts.reshape(-1, 2)


def _rmse(proj, ref, inliers=None):
    proj = np.asarray(proj, np.float64).reshape(-1, 2)
    ref  = np.asarray(ref,  np.float64).reshape(-1, 2)
    if inliers is not None:
        idx = np.asarray(inliers).reshape(-1)
        proj, ref = proj[idx], ref[idx]
    return float(np.sqrt(np.mean(np.sum((proj - ref) ** 2, axis=1))))


# ── PCD save ──────────────────────────────────────────────────────────────────

def _save_pcd(pts_xyz, path: str):
    n = len(pts_xyz)
    with open(path, 'w') as f:
        f.write(f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n')
        for p in pts_xyz:
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')


# ── Interactive pickers (run inside subprocesses) ─────────────────────────────

def _pick_2d(img_bgr, now_sec, dataset_path, camera_id):
    """Matplotlib-based 2D corner picker. Must run in its own process."""
    import matplotlib.pyplot as plt
    from matplotlib import use as mpl_use
    mpl_use('TkAgg')

    disp = img_bgr[:, :, ::-1]
    fig, ax = plt.subplots(figsize=(14, 7))
    ax.set_title(f't={now_sec}  Click 4 corners (same order as 3D picker). [r]=reset')
    ax.set_axis_off()
    ax.imshow(disp)

    pts, lines = [], []

    def onclick(ev):
        if ev.xdata is None:
            return
        pts.append((ev.xdata, ev.ydata))
        if len(pts) > 1:
            arr = np.array(pts)
            l, = ax.plot(arr[:, 0], arr[:, 1], 'r-o', markersize=5)
            lines.append(l)
            fig.canvas.draw_idle()

    def onkey(ev):
        if ev.key == 'r':
            for l in lines:
                l.remove()
            lines.clear()
            pts.clear()
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('button_press_event', onclick)
    fig.canvas.mpl_connect('key_press_event', onkey)
    plt.tight_layout()
    plt.show()

    if len(pts) == 5:
        pts.pop()
    if len(pts) != 4:
        print(f'[2D] Need 4 corners, got {len(pts)}. Frame skipped.')
        return

    img_dir  = Path(dataset_path) / f'images_{camera_id}'
    img_dir.mkdir(parents=True, exist_ok=True)
    csv_path = Path(dataset_path) / f'2D_corners_{camera_id}.csv'

    existing = (np.loadtxt(str(csv_path), delimiter=',').reshape(-1, 2).tolist()
                if csv_path.exists() else [])
    existing.extend(pts)
    np.savetxt(str(csv_path), np.array(existing).reshape(-1, 2), delimiter=',')

    count = len(list(img_dir.glob('*.jpg')))
    cv2.imwrite(str(img_dir / f'{count:02d}.jpg'), img_bgr)
    print(f'[2D] Saved {count:02d}.jpg + {len(pts)} corners → total {len(existing)//4} sets')


def _pick_3d(pts_np, now_sec, dataset_path, camera_id):
    """Matplotlib-based 3D point picker. Must run in its own process."""
    import matplotlib.pyplot as plt
    from matplotlib import use as mpl_use
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    mpl_use('TkAgg')

    mask = ((pts_np[:, 0] > ROI['x_min']) & (pts_np[:, 0] < ROI['x_max']) &
            (pts_np[:, 1] > ROI['y_min']) & (pts_np[:, 1] < ROI['y_max']) &
            (pts_np[:, 2] > ROI['z_min']) & (pts_np[:, 2] < ROI['z_max']))
    pts = pts_np[mask]
    if len(pts) < 5:
        print('[3D] No points in ROI. Adjust ROI or move sensor.')
        return

    intens = pts[:, 3] if pts.shape[1] > 3 else np.zeros(len(pts))
    cmap   = matplotlib.colormaps.get_cmap('hsv')
    colors = cmap(intens / (intens.max() + 1e-6))

    fig = plt.figure(figsize=(14, 7))
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_title(f't={now_sec}  Click 4 LiDAR points matching the image corners. [r]=reset')
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=colors, s=5, picker=5)

    picked, lines = [], []

    def onpick(ev):
        ind = ev.ind[0]
        x, y, z = float(pts[ind, 0]), float(pts[ind, 1]), float(pts[ind, 2])
        picked.append((x, y, z))
        print(f'[3D] Picked #{len(picked)}: ({x:.3f}, {y:.3f}, {z:.3f})')
        if len(picked) > 1:
            arr = np.array(picked)
            l, = ax.plot(arr[:, 0], arr[:, 1], arr[:, 2])
            lines.append(l)
            fig.canvas.draw_idle()

    def onkey(ev):
        if ev.key == 'r':
            for l in lines:
                l.remove()
            lines.clear()
            picked.clear()
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('pick_event', onpick)
    fig.canvas.mpl_connect('key_press_event', onkey)
    plt.tight_layout()
    plt.show()

    if len(picked) == 5:
        picked.pop()
    if len(picked) != 4:
        print(f'[3D] Need 4 points, got {len(picked)}. Frame skipped.')
        return

    pcd_dir  = Path(dataset_path) / f'pointclouds_{camera_id}'
    pcd_dir.mkdir(parents=True, exist_ok=True)
    csv_path = Path(dataset_path) / f'3D_corners_{camera_id}.csv'

    existing = (np.loadtxt(str(csv_path), delimiter=',').reshape(-1, 3).tolist()
                if csv_path.exists() else [])
    existing.extend(picked)
    np.savetxt(str(csv_path), np.array(existing).reshape(-1, 3), delimiter=',')

    count = len(list(pcd_dir.glob('*.pcd')))
    _save_pcd(pts[:, :3], str(pcd_dir / f'{count:02d}.pcd'))
    print(f'[3D] Saved {count:02d}.pcd + {len(picked)} corners → total {len(existing)//3} sets')


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class ExtrinsicCalibNode(Node):

    def __init__(self, params: dict):
        super().__init__('extrinsic_calib_node')
        self._p          = params
        self._lock       = threading.Lock()
        self._frame      = None
        self._cam_params = None

        name       = params['camera_id']
        output_dir = Path(os.path.expanduser(params['output_dir']))

        # ── Intrinsics source (priority order) ───────────────────────────────
        yaml_path    = params.get('camera_info_yaml', '')
        cv_yml_path  = output_dir / f'{name}_intrinsic.yml'
        ros_yml_path = Path.home() / '.ros' / 'camera_info' / f'camera_{name}.yaml'

        if yaml_path and Path(yaml_path).exists():
            self._cam_params = _load_intrinsics_yaml(yaml_path)
            self.get_logger().info(f'Intrinsics loaded from (explicit): {yaml_path}')
        elif cv_yml_path.exists():
            try:
                self._cam_params = _load_intrinsics_cv_yml(str(cv_yml_path))
                self.get_logger().info(f'Intrinsics loaded from (OpenCV YML): {cv_yml_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load {cv_yml_path}: {e}')
        elif ros_yml_path.exists():
            try:
                self._cam_params = _load_intrinsics_yaml(str(ros_yml_path))
                self.get_logger().info(f'Intrinsics auto-loaded from (ROS): {ros_yml_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load {ros_yml_path}: {e}')

        if self._cam_params is None:
            self._info_sub = self.create_subscription(
                CameraInfo, params['camera_info_topic'], self._cam_info_cb, 1)
            self.get_logger().info(
                f'Waiting for CameraInfo on {params["camera_info_topic"]} ...')

        # ── Synced image + point cloud ────────────────────────────────────────
        img_sub = message_filters.Subscriber(self, Image,       params['image_topic'])
        pc_sub  = message_filters.Subscriber(self, PointCloud2, params['lidar_topic'])
        ats = message_filters.ApproximateTimeSynchronizer(
            [img_sub, pc_sub], queue_size=5, slop=0.1)
        ats.registerCallback(self._frame_cb)
        self.get_logger().info('Subscribed to image + point_cloud. Ready.')

    def _cam_info_cb(self, msg: CameraInfo):
        if self._cam_params is None:
            self._cam_params = _cam_params_from_info_msg(msg)
            self.get_logger().info('CameraInfo received — intrinsics ready.')

    def _frame_cb(self, img_msg: Image, pc_msg: PointCloud2):
        img_bgr = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
        pts_gen = pc2_util.read_points(
            pc_msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
        pts_arr = np.array(list(pts_gen))   # structured dtype: fields x,y,z,intensity
        if pts_arr.size == 0:
            pts_np = np.empty((0, 4), dtype=np.float32)
        else:
            pts_np = np.column_stack([
                pts_arr['x'], pts_arr['y'], pts_arr['z'], pts_arr['intensity']
            ]).astype(np.float32)
        now_sec = img_msg.header.stamp.sec
        with self._lock:
            self._frame = (img_bgr, pts_np, now_sec)

    def get_frame(self):
        with self._lock:
            return self._frame

    # ── Calibrate ─────────────────────────────────────────────────────────────

    def calibrate(self) -> bool:
        p    = self._p
        name = p['camera_id']
        dp   = Path(p['dataset_path'])

        c2d = dp / f'2D_corners_{name}.csv'
        c3d = dp / f'3D_corners_{name}.csv'
        if not c2d.exists() or not c3d.exists():
            self.get_logger().error('No stored corners. Press [s] first.')
            return False

        corners_2d = np.loadtxt(str(c2d), delimiter=',').reshape(-1, 2)
        corners_3d = np.loadtxt(str(c3d), delimiter=',').reshape(-1, 3)

        if self._cam_params is None:
            self.get_logger().error(
                'No camera intrinsics. Check CameraInfo topic or --camera-info-yaml.')
            return False

        cam = self._cam_params
        n   = len(corners_2d)
        self.get_logger().info(
            f'Calibrating with {n} point pairs '
            f'({cam["camera_model"]}, {cam["distortion_model"]})')

        ok, r, t, inliers = _solve_pnp(cam, corners_3d, corners_2d)
        if not ok:
            self.get_logger().error('solvePnPRansac failed.')
            return False

        reproj = _project_points(cam, corners_3d, r, t)
        rmse   = _rmse(reproj, corners_2d, inliers)
        n_in   = 0 if inliers is None else len(np.asarray(inliers).reshape(-1))
        self.get_logger().info(f'Inliers: {n_in}/{n}   RMSE: {rmse:.2f} px')

        R_mat = cv2.Rodrigues(r)[0]            # 3×3
        t_col = t.reshape(3, 1)               # 3×1

        out = Path(os.path.expanduser(p['output_dir']))
        out.mkdir(parents=True, exist_ok=True)

        # x_cam = R · x_lidar + t  (LiDAR frame → Camera frame)
        extr_path = out / f'{name}_extrinsic.yml'
        fs = cv2.FileStorage(str(extr_path), cv2.FILE_STORAGE_WRITE)
        fs.write('camera_id', name)
        fs.write('R', R_mat)
        fs.write('t', t_col)
        fs.release()
        self.get_logger().info(f'[Extrinsic YML saved] {extr_path}')

        # ── Save camera intrinsics alongside ─────────────────────────────────
        K  = cam['camera_matrix']
        D  = cam['dist_coeffs'].flatten()
        P  = cam['projection_matrix']
        cy = {
            'image_width':      cam['image_width'],
            'image_height':     cam['image_height'],
            'camera_model':     cam['camera_model'],
            'distortion_model': cam['distortion_model'],
            'K': K.flatten().tolist(),
            'D': D.tolist(),
            'fx': float(K[0, 0]),  'fy': float(K[1, 1]),
            'cx': float(K[0, 2]),  'cy': float(K[1, 2]),
            'fx_rectified': float(P[0, 0]), 'fy_rectified': float(P[1, 1]),
            'cx_rectified': float(P[0, 2]), 'cy_rectified': float(P[1, 2]),
            'rmse_px': round(float(rmse), 4),
        }
        cam_path = out / f'camera_info_{name}.yaml'
        cam_path.write_text(yaml.dump(cy, default_flow_style=False))
        self.get_logger().info(f'[Cam info saved]      {cam_path}')
        return True

    # ── Reset ─────────────────────────────────────────────────────────────────

    def reset(self):
        import shutil
        dp = Path(self._p['dataset_path'])
        if dp.exists():
            shutil.rmtree(str(dp))
        self.get_logger().info(f'Dataset reset: {dp}')


# ── Keyboard handler ──────────────────────────────────────────────────────────

def _keyboard_thread(stop_event: threading.Event):
    global _g_pause, _g_store, _g_calib, _g_reset

    # ros2 launch replaces stdin with a pipe — open /dev/tty to reach the terminal
    try:
        tty_file = open('/dev/tty', 'r')
    except OSError:
        tty_file = sys.stdin

    while not stop_event.is_set():
        sys.stdout.write('[s]=store  [c]=calibrate  [r]=reset  [q]=quit\n> ')
        sys.stdout.flush()
        try:
            line = tty_file.readline()
            if not line:
                break
            mode = line.strip().lower()
        except (EOFError, OSError):
            break
        with _g_lock:
            if   mode == 's':
                _g_store = True
                _g_pause = True
            elif mode == 'c':
                _g_calib = True
                _g_pause = True
            elif mode == 'r':
                _g_reset = True
                _g_pause = True
            elif mode == 'q':
                stop_event.set()

    if tty_file is not sys.stdin:
        tty_file.close()


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    global _g_pause, _g_store, _g_calib, _g_reset

    ap = argparse.ArgumentParser(
        description='NSL-3130AA extrinsic calibration (ROS2 Humble)')
    ap.add_argument('--camera-id',         default='nsl',
                    help='Camera ID (used in output filenames)')
    ap.add_argument('--image-topic',        default='/camera/rgb/image_raw')
    ap.add_argument('--camera-info-topic',  default='/camera/rgb/camera_info')
    ap.add_argument('--camera-info-yaml',   default='',
                    help='Explicit path to camera intrinsics YAML (skips other sources)')
    ap.add_argument('--lidar-topic',        default='/camera/point_cloud')
    ap.add_argument('--output-dir',
                    default=os.path.join(os.getcwd(), 'calib_output'),
                    help='Directory for output files')
    args, ros_args = ap.parse_known_args()

    camera_id   = args.camera_id
    output_dir  = os.path.expanduser(args.output_dir)
    dataset_path = os.path.join(output_dir, f'extrinsic_{camera_id}')

    # Guard: intrinsic file must exist before extrinsic calibration
    intr_file = Path(output_dir) / f'{camera_id}_intrinsic.yml'
    if not intr_file.exists():
        candidates = sorted(Path(output_dir).glob('*_intrinsic.yml')) if Path(output_dir).exists() else []
        print(f'\n[ERROR] Intrinsic file not found: {intr_file}')
        if candidates:
            detected_id = candidates[0].stem.replace('_intrinsic', '')
            print(f'        Found: {candidates[0]}')
            print(f'        Re-run with the correct camera_id:')
            print(f'          ./extrinsic_calib.sh {detected_id}')
        else:
            print(f'        No intrinsic files found in: {output_dir}')
            print(f'        Run intrinsic calibration first:')
            print(f'          ./intrinsic_calib.sh')
        print()
        sys.exit(1)

    params = {
        'camera_id':          camera_id,
        'image_topic':        args.image_topic,
        'camera_info_topic':  args.camera_info_topic,
        'camera_info_yaml':   args.camera_info_yaml,
        'lidar_topic':        args.lidar_topic,
        'dataset_path':       dataset_path,
        'output_dir':         output_dir,
    }

    rclpy.init(args=ros_args or None)
    node = ExtrinsicCalibNode(params)

    stop_ev = threading.Event()
    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()
    kb_th   = threading.Thread(target=_keyboard_thread, args=(stop_ev,), daemon=True)
    kb_th.start()

    import time
    try:
        while rclpy.ok() and not stop_ev.is_set():
            with _g_lock:
                act = _g_pause

            if not act:
                time.sleep(0.05)
                continue

            with _g_lock:
                do_store = _g_store
                do_calib = _g_calib
                do_reset = _g_reset
                _g_store = _g_calib = _g_reset = _g_pause = False

            if do_store:
                frame = node.get_frame()
                if frame is None:
                    print('[WARN] No frame received yet. Waiting for camera + lidar...')
                else:
                    img_bgr, pts_np, now_sec = frame
                    p2d = multiprocessing.Process(
                        target=_pick_2d,
                        args=(img_bgr, now_sec, params['dataset_path'], camera_id))
                    p3d = multiprocessing.Process(
                        target=_pick_3d,
                        args=(pts_np, now_sec, params['dataset_path'], camera_id))
                    p2d.start()
                    p3d.start()
                    p2d.join()
                    p3d.join()

            if do_calib:
                node.calibrate()

            if do_reset:
                node.reset()

    except KeyboardInterrupt:
        pass
    finally:
        stop_ev.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    multiprocessing.set_start_method('spawn')
    main()
