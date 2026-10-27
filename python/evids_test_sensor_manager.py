# EViDS sensor callback managers.

import numpy as np
import os
import cv2  # pip install opencv-python
import threading
import time
# evids_test_initialize_python.m loads the user's installed CARLA client or external egg.
import carla

class SensorManager:
    """
    一个通用的传感器管理器，用于处理基于颜色的图像数据。
    它接收CARLA的图像数据并将其存储为一个Numpy数组。
    """
    def __init__(self, width, height, channels=3, dtype=np.uint8):
        """
        初始化一个占位符数组，以防止在第一帧数据到达前访问属性时出错。
        """
        # 注意：最终输出通道数为3 (BGR)
        self.array = np.zeros((height, width, 3), dtype=dtype)

    def __call__(self, carla_image):
        """
        这个方法将作为传感器的回调函数。
        当实例本身被调用时（例如在.listen()中），此方法会执行。
        """
        # 将CARLA图像的原始数据缓冲区转换为一个Numpy数组
        # CARLA的摄像头数据格式为 BGRA (蓝、绿、红、透明度)
        array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
        
        # 将一维数组重塑为4通道图像 (height, width, 4)
        array = np.reshape(array, (carla_image.height, carla_image.width, 4))
        
        # 仅提取BGR三个颜色通道，忽略Alpha通道
        self.array = array[:, :, :3]


class DepthSensorManager:
    """
    一个专门用于处理深度摄像头数据的传感器管理器
    """
    def __init__(self, width, height, dtype=np.float64):
        """
        初始化一个占位符数组。
        """
        self.array = np.zeros((height, width), dtype=dtype)

    def __call__(self, carla_image):
        """
        作为深度传感器的回调函数。
        将CARLA的深度数据编码在颜色通道中转换为一个归一化的深度图。
        """
        # 将CARLA图像的原始数据缓冲区转换为一个Numpy数组
        array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (carla_image.height, carla_image.width, 4))
        
        # 将BGRA数据转换为浮点数
        array = array.astype(np.float64)
        
        # 应用CARLA官方公式将编码后的颜色转换回[0, 1]范围内的深度值
        # normalized_depth = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
        # 注意：数组索引中 R=2, G=1, B=0
        normalized_depth = (array[:, :, 2] + array[:, :, 1] * 256 + array[:, :, 0] * 256 * 256) / (256 * 256 * 256 - 1)
        
        # 将计算出的深度图存储起来
        self.array = normalized_depth

class LaneInvasionManager:
    """
    用于处理 CARLA 的 sensor.other.lane_invasion 事件。
    这个类是可调用的，可以直接作为 lane_sensor.listen(LaneInvasionManager(...))
    - 它会保存最近一次事件信息到 self.last_event
    - 并提供一个简单的回调友好打印和可被外部查询的结构
    """
    def __init__(self, verbose=True):
        self.verbose = verbose
        self.lock = threading.Lock()
        self.last_event = None  # 存放最近一次 event 对象
        self.history = []       # 可选：存放简短历史

    def _marking_to_str(self, marking):
        # 尽量返回可读的类型名称
        try:
            tname = marking.type.name  # 大多数 CARLA 版本支持
        except Exception:
            tname = str(marking.type)
        return f"type={tname}, id={getattr(marking, 'id', 'N/A')}"

    def __call__(self, event):
        """
        event 是 carla.LaneInvasionEvent
        event.crossed_lane_markings 是一个 LaneMarking 的 list（可能为空）
        event.crossed_lane_markings 中的每个元素含 type 和 id 等属性
        """
        with self.lock:
            info = {
                'timestamp': time.time(),
                'num_crossed': len(event.crossed_lane_markings),
                'markings': [self._marking_to_str(m) for m in event.crossed_lane_markings]
            }
            self.last_event = info
            self.history.append(info)
            # 控制 history 长度，防止无限增长
            if len(self.history) > 2000:
                self.history.pop(0)

        if self.verbose:
            print("=== LaneInvasionEvent ===")
            print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(info['timestamp']))}")
            if info['num_crossed'] == 0:
                print("Crossed lane markings: NONE")
            else:
                for i, s in enumerate(info['markings']):
                    print(f"  Marking[{i}]: {s}")
                    
            print("=========================")

    def get_last(self):
        with self.lock:
            return None if self.last_event is None else dict(self.last_event)

    def clear_history(self):
        with self.lock:
            self.history = []
            self.last_event = None


class VisionLaneDetector:
    """
    基于 RGB 相机（sensor.camera.rgb） + OpenCV 的傻瓜车道线检测。
    使用方法示例:
        vision = VisionLaneDetector(width, height)
        camera_sensor.listen(vision)    # camera_sensor 为 CARLA 相机 actor
    回调收到的是 carla.Image，VisionLaneDetector 会保存:
      - self.last_overlay: 带检测线的 BGR numpy 图片
      - self.last_lines: Hough 检测到的线段列表
    注意：这是基础实现，适合“开发原型/可视化/参数调试”。
    """
    def __init__(self, width, height,
                 canny_low=50, canny_high=150,
                 hough_rho=1, hough_theta=np.pi/180, hough_threshold=30,
                 min_line_len=20, max_line_gap=20,
                 region_fraction=(0.55, 1.0)):  # region_fraction=(y_start_fraction, y_end_fraction)
        self.w = width
        self.h = height
        self.canny_low = canny_low
        self.canny_high = canny_high
        self.hough_rho = hough_rho
        self.hough_theta = hough_theta
        self.hough_threshold = hough_threshold
        self.min_line_len = min_line_len
        self.max_line_gap = max_line_gap
        self.region_fraction = region_fraction

        # 存放最近结果，供外部查询
        self.last_overlay = np.zeros((height, width, 3), dtype=np.uint8)
        self.last_lines = None
        self.lock = threading.Lock()

    def _region_of_interest(self, img):
        y1 = int(self.h * self.region_fraction[0])
        y2 = int(self.h * self.region_fraction[1])
        mask = np.zeros_like(img)
        # 多边形遮罩 - 底部中央三角形区域
        polygon = np.array([[
            (0, self.h),
            (0, y1),
            (self.w, y1),
            (self.w, self.h)
        ]], dtype=np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked = cv2.bitwise_and(img, mask)
        return masked

    def __call__(self, carla_image):
        # 接收 carla.Image，先转为 numpy BGR（与上面 SensorManager 风格一致）
        array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (carla_image.height, carla_image.width, 4))
        bgr = array[:, :, :3].copy()

        # 1. 灰度 -> Canny -> ROI -> HoughLinesP
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, self.canny_low, self.canny_high)

        roi = self._region_of_interest(edges)
        lines = cv2.HoughLinesP(roi,
                                rho=self.hough_rho,
                                theta=self.hough_theta,
                                threshold=self.hough_threshold,
                                minLineLength=self.min_line_len,
                                maxLineGap=self.max_line_gap)
        overlay = bgr.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # 画线（厚度 2）
                cv2.line(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 保存结果（线程安全）
        with self.lock:
            self.last_overlay = overlay
            # 转为更紧凑的表示：列表 of tuples
            if lines is None:
                self.last_lines = []
            else:
                self.last_lines = [(int(l[0][0]), int(l[0][1]), int(l[0][2]), int(l[0][3])) for l in lines]

    def get_overlay(self):
        with self.lock:
            return self.last_overlay.copy()

    def get_lines(self):
        with self.lock:
            return list(self.last_lines)


class LidarSensorManager:
    """
    CARLA LiDAR 回调管理器（可直接传入 actor.listen()）
    将每帧点云解析为 numpy.ndarray shape=(N,4) (x,y,z,intensity)
    线程安全存取：
      - get_point_cloud() -> 返回 (N,4) numpy 副本
      - save_to_pcd(path)  -> 将最近一帧点云保存为 ASCII PCD
      - save_to_npy(path)  -> 将最近一帧点云保存为 .npy (np.save)
    使用示例:
      lidar_module = LidarSensorManager()
      lidar_actor.listen(lidar_module)
      # 之后可以在 MATLAB / Python 循环里:
      pts = lidar_module.get_point_cloud()
      lidar_module.save_to_pcd('.../frame0001.pcd')
    """
    def __init__(self, invert_y: bool = True, max_points: int = 1000000):
        """
        invert_y: 是否把 y 轴取反（与 sample script 保持一致）
        max_points: 预分配或作为保护性上限（不会截断真实收到的数据，但可用于提示）
        """
        self.invert_y = bool(invert_y)
        self.max_points = int(max_points)
        self.lock = threading.Lock()
        # 存储最近一帧点云 (N,4) float32
        self._points = np.zeros((0,4), dtype=np.float32)
        self.last_timestamp = None
        self.frame = 0

    def __call__(self, carla_lidar):
        """
        carla_lidar: carla.LidarMeasurement
        raw_data 是连续的 float32 数组：x,y,z,intensity,x,y,z,intensity...
        """
        # 解析 raw_data 为 float32 数组
        data = np.frombuffer(carla_lidar.raw_data, dtype=np.float32)
        if data.size == 0:
            with self.lock:
                self._points = np.zeros((0,4), dtype=np.float32)
                self.last_timestamp = time.time()
            return

        try:
            points = data.reshape((-1, 4))
        except Exception:
            # 若 reshape 失败，则直接置空并返回
            with self.lock:
                self._points = np.zeros((0,4), dtype=np.float32)
                self.last_timestamp = time.time()
            return

        # 与你给的示例一致：把 y 轴取反（某些 CARLA 帧坐标系约定）
        if self.invert_y and points.shape[0] > 0:
            points = points.copy()
            points[:, 1] = -points[:, 1]

        with self.lock:
            self._points = points.astype(np.float32, copy=True)
            self.last_timestamp = time.time()
            self.frame += 1

    def get_point_cloud(self):
        """
        返回最近一帧的点云副本，shape=(N,4)
        """
        with self.lock:
            return self._points.copy()

    def get_xyz(self):
        """
        返回 (N,3) 的 xyz 副本（丢弃 intensity）
        """
        with self.lock:
            if self._points.shape[0] == 0:
                return np.zeros((0,3), dtype=np.float32)
            return self._points[:, :3].copy()

    def save_to_npy(self, path):
        """
        快速保存为 .npy (包含 intensity)；path: 字符串（文件路径）
        """
        # 确保目录存在
        d = os.path.dirname(path)
        if d and not os.path.exists(d):
            os.makedirs(d, exist_ok=True)
        pts = self.get_point_cloud()
        # np.save 会自动追加 .npy 扩展名（如果用户未写）
        np.save(path, pts)

    def save_to_pcd(self, path):
        """
        将最近一帧点云保存为 ASCII PCD（fields: x y z intensity）
        兼容大多数点云可视化工具（PCL, CloudCompare, ROS pcd reader 等）
        注意：当点数非常多时，这会生成很大的文本文件，.npy 更节约空间。
        """
        pts = self.get_point_cloud()
        n = pts.shape[0]
        d = os.path.dirname(path)
        if d and not os.path.exists(d):
            os.makedirs(d, exist_ok=True)

        header = [
            '# .PCD v0.7 - Point Cloud Data file format',
            'VERSION 0.7',
            'FIELDS x y z intensity',
            'SIZE 4 4 4 4',
            'TYPE F F F F',
            'COUNT 1 1 1 1',
            f'WIDTH {n}',
            'HEIGHT 1',
            f'POINTS {n}',
            'DATA ascii'
        ]
        try:
            with open(path, 'w') as f:
                f.write("\n".join(header) + "\n")
                # 写入每行：x y z intensity
                # 使用高效逐行写入
                for i in range(n):
                    x, y, z, it = pts[i]
                    f.write(f"{x:.6f} {y:.6f} {z:.6f} {it:.6f}\n")
        except Exception as e:
            # 若文件写入失败，抛出异常以便上层捕获 / 日志
            raise

    # 可选：把点云作为 open3d 点云返回（如果系统安装了 open3d）
    def to_open3d(self):
        try:
            import open3d as o3d
        except Exception:
            raise RuntimeError("open3d not available in Python environment")
        pts = self.get_point_cloud()
        if pts.shape[0] == 0:
            return o3d.geometry.PointCloud()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts[:, :3])
        # use intensity as colors (normalized)
        inten = pts[:, 3]
        if np.max(inten) > 0:
            inten_norm = inten / np.max(inten)
            colors = np.vstack([inten_norm, inten_norm, inten_norm]).T
            pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd


def _vehicle_box(actor, transform):
    """World OBB, including actor and bounding-box rotations."""
    box = actor.bounding_box
    matrix = np.asarray(transform.get_matrix(), dtype=float)
    local = np.array([box.location.x, box.location.y, box.location.z, 1.0])
    center = np.dot(matrix, local)[:3]
    rotation = getattr(box, 'rotation', carla.Rotation())
    box_matrix = np.asarray(carla.Transform(carla.Location(), rotation).get_matrix(), dtype=float)
    axes = np.dot(matrix[:3, :3], box_matrix[:3, :3])
    extent = np.array([box.extent.x, box.extent.y, box.extent.z], dtype=float)
    return center, axes, extent


def _vehicle_boxes_overlap(first, second):
    """Separating-axis test; touching boundaries alone are not overlap."""
    center_a, axes_a, extent_a = first
    center_b, axes_b, extent_b = second
    candidates = [axes_a[:, i] for i in range(3)] + [axes_b[:, i] for i in range(3)]
    candidates += [np.cross(axes_a[:, i], axes_b[:, j]) for i in range(3) for j in range(3)]
    delta = center_b - center_a
    for axis in candidates:
        if np.linalg.norm(axis) < 1e-9:
            continue
        radius_a = np.dot(extent_a, np.abs(np.dot(axes_a.T, axis)))
        radius_b = np.dot(extent_b, np.abs(np.dot(axes_b.T, axis)))
        if abs(np.dot(delta, axis)) >= radius_a + radius_b - 1e-6:
            return False
    return True


def spawn_ghost_vehicle(world, blueprint, target, diagnostics_path=''):
    """Spawn a kinematic visualization vehicle without changing the target pose.

    A CarSim reference origin at road height can fail CARLA's initial collision
    check. Try a deterministic elevated staging pose, disable physics, check
    existing vehicles at the target, then teleport to the exact requested pose.
    No pre-existing actor is destroyed, and no world settings are changed here.
    """
    import json
    from pathlib import Path
    actor = None
    record = {'requested_pose': {
        'x': target.location.x, 'y': target.location.y, 'z': target.location.z,
        'pitch': target.rotation.pitch, 'roll': target.rotation.roll, 'yaw': target.rotation.yaw},
        'attempts': [], 'status': 'starting'}

    def save_record():
        if diagnostics_path:
            path = Path(str(diagnostics_path))
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(record, indent=2), encoding='utf-8')

    try:
        actor = world.try_spawn_actor(blueprint, target)
        record['attempts'].append({'z': target.location.z, 'success': actor is not None})
        if actor is not None:
            actor.set_simulate_physics(False)
            record.update(status='success', method='direct', actor_id=actor.id)
            save_record()
            return actor

        # Road height is used only to find staging clearance, never to rewrite
        # CarSim pose, tire positions or the four-wheel road-feedback outputs.
        top = max(float(target.location.z) + 50.0, 50.0)
        bottom = float(target.location.z) - 50.0
        hits = world.cast_ray(carla.Location(target.location.x, target.location.y, top),
                              carla.Location(target.location.x, target.location.y, bottom))
        road = [float(h.location.z) for h in hits if int(h.label) in (6, 7, 8)]
        road_z = max(road) if road else float(target.location.z)
        record['staging_road_z'] = road_z
        base_z = max(float(target.location.z), road_z)
        for clearance in (0.5, 1.0, 2.0, 4.0, 8.0):
            staging = carla.Transform(
                carla.Location(target.location.x, target.location.y, base_z + clearance),
                target.rotation)
            actor = world.try_spawn_actor(blueprint, staging)
            record['attempts'].append({'z': staging.location.z, 'success': actor is not None})
            if actor is not None:
                break
        if actor is None:
            raise RuntimeError('No free staging position above the requested XY. '
                               'See spawn_diagnostics.json for the requested pose and attempted heights.')
        actor.set_simulate_physics(False)
        target_box = _vehicle_box(actor, target)
        for other in world.get_actors().filter('vehicle.*'):
            if other.id != actor.id and other.is_alive:
                if _vehicle_boxes_overlap(target_box, _vehicle_box(other, other.get_transform())):
                    record['occupied_by_actor'] = other.id
                    raise RuntimeError('The target vehicle footprint is occupied by actor %d. '
                                       'No existing vehicles were removed.' % other.id)
        actor.set_transform(target)
        record.update(status='success', method='elevated_then_teleported', actor_id=actor.id)
        save_record()
        print('EVIDS ghost vehicle created via elevated staging; exact CarSim pose preserved: %s' % record['requested_pose'])
        return actor
    except Exception as exc:
        if actor is not None and actor.is_alive:
            actor.destroy()
        record.update(status='failed', error=str(exc))
        save_record()
        raise
