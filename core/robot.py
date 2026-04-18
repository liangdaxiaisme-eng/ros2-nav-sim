"""
机器人仿真器
- 差速驱动运动模型
- 模拟激光雷达 (LIDAR)
- 模拟里程计 (Odometry)
- 模拟 IMU
"""
import numpy as np
import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import time


@dataclass
class RobotState:
    x: float = 0.0        # 世界坐标 (m)
    y: float = 0.0
    theta: float = 0.0    # 航向角 (rad)
    v: float = 0.0        # 线速度 (m/s)
    w: float = 0.0        # 角速度 (rad/s)


@dataclass
class LaserScan:
    """模拟激光雷达数据"""
    angle_min: float = 0.0
    angle_max: float = 2 * math.pi
    angle_increment: float = math.pi / 180  # 1度分辨率
    range_min: float = 0.1
    range_max: float = 10.0
    ranges: List[float] = field(default_factory=list)
    intensities: List[float] = field(default_factory=list)
    timestamp: float = 0.0


@dataclass
class OdometryData:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    w: float = 0.0
    timestamp: float = 0.0


class DifferentialDriveRobot:
    """差速驱动机器人模型"""

    def __init__(self, x: float, y: float, theta: float = 0.0,
                 wheel_base: float = 0.5, wheel_radius: float = 0.1):
        self.state = RobotState(x, y, theta)
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.max_linear_vel = 1.5  # m/s
        self.max_angular_vel = 3.0  # rad/s

        # 噪声参数
        self.odom_noise_lin = 0.02
        self.odom_noise_ang = 0.01

        # 历史轨迹
        self.trajectory: List[Tuple[float, float, float]] = []
        self.trajectory_max_len = 2000

        # 速度指令历史
        self.cmd_vel_history: List[Tuple[float, float, float]] = []

    def update(self, v_cmd: float, w_cmd: float, dt: float) -> RobotState:
        """更新机器人状态"""
        # 限速
        v_cmd = max(self.max_linear_vel, min(-self.max_linear_vel, v_cmd))
        w_cmd = max(self.max_angular_vel, min(-self.max_angular_vel, w_cmd))

        # 添加噪声
        v_noise = np.random.normal(0, self.odom_noise_lin)
        w_noise = np.random.normal(0, self.odom_noise_ang)

        v_actual = v_cmd + v_noise
        w_actual = w_cmd + w_noise

        # 运动学更新
        self.state.theta += w_actual * dt
        self.state.theta = math.atan2(math.sin(self.state.theta),
                                       math.cos(self.state.theta))
        self.state.x += v_actual * math.cos(self.state.theta) * dt
        self.state.y += v_actual * math.sin(self.state.theta) * dt
        self.state.v = v_actual
        self.state.w = w_actual

        # 记录轨迹
        self.trajectory.append((self.state.x, self.state.y, self.state.theta))
        if len(self.trajectory) > self.trajectory_max_len:
            self.trajectory.pop(0)

        self.cmd_vel_history.append((v_cmd, w_cmd, time.time()))
        if len(self.cmd_vel_history) > 500:
            self.cmd_vel_history.pop(0)

        return self.state

    def get_laser_scan(self, obstacle_mask: np.ndarray, resolution: float,
                       num_rays: int = 360, max_range: float = 10.0) -> LaserScan:
        """模拟 360° 激光雷达"""
        scan = LaserScan(
            angle_min=0,
            angle_max=2 * math.pi,
            angle_increment=2 * math.pi / num_rays,
            range_max=max_range
        )

        px = int(self.state.x / resolution)
        py = int(self.state.y / resolution)
        height, width = obstacle_mask.shape

        for i in range(num_rays):
            angle = self.state.theta + i * scan.angle_increment
            dist = self._raycast(px, py, angle, obstacle_mask, resolution, max_range)
            scan.ranges.append(dist)
            # 模拟强度 (近处高)
            scan.intensities.append(max(0, 1.0 - dist / max_range) if dist < max_range else 0.0)

        scan.timestamp = time.time()
        return scan

    def _raycast(self, px: int, py: int, angle: float, obstacle_mask: np.ndarray,
                 resolution: float, max_range: float) -> float:
        """射线投射"""
        dx = math.cos(angle)
        dy = math.sin(angle)
        height, width = obstacle_mask.shape

        dist = 0.0
        step = resolution * 0.5  # 半格步长

        while dist < max_range:
            cx = int(px + dx * dist / resolution)
            cy = int(py + dy * dist / resolution)

            if cx < 0 or cx >= width or cy < 0 or cy >= height:
                return max_range

            if obstacle_mask[cy, cx]:
                # 加点噪声
                return dist + np.random.normal(0, 0.02)

            dist += step

        return max_range

    def get_odometry(self) -> OdometryData:
        """获取里程计数据 (含噪声)"""
        return OdometryData(
            x=self.state.x + np.random.normal(0, 0.01),
            y=self.state.y + np.random.normal(0, 0.01),
            theta=self.state.theta + np.random.normal(0, 0.005),
            v=self.state.v,
            w=self.state.w,
            timestamp=time.time()
        )

    def get_imu_data(self) -> dict:
        """模拟 IMU 数据"""
        return {
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': math.sin(self.state.theta / 2),
                'w': math.cos(self.state.theta / 2),
            },
            'angular_velocity': {
                'x': np.random.normal(0, 0.01),
                'y': np.random.normal(0, 0.01),
                'z': self.state.w + np.random.normal(0, 0.02),
            },
            'linear_acceleration': {
                'x': np.random.normal(0, 0.1),
                'y': np.random.normal(0, 0.1),
                'z': 9.81 + np.random.normal(0, 0.05),
            },
            'timestamp': time.time()
        }

    def check_collision(self, obstacle_mask: np.ndarray, resolution: float,
                        robot_radius_px: int = 5) -> bool:
        """碰撞检测"""
        px = int(self.state.x / resolution)
        py = int(self.state.y / resolution)
        height, width = obstacle_mask.shape

        for dx in range(-robot_radius_px, robot_radius_px + 1):
            for dy in range(-robot_radius_px, robot_radius_px + 1):
                if dx * dx + dy * dy <= robot_radius_px * robot_radius_px:
                    cx, cy = px + dx, py + dy
                    if 0 <= cx < width and 0 <= cy < height:
                        if obstacle_mask[cy, cx]:
                            return True
        return False
