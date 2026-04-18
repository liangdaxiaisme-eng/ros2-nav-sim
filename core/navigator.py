"""
航点导航控制器
- 航点序列管理
- 目标跟踪
- 状态机 (IDLE → PLANNING → NAVIGATING → ARRIVED)
- 异常处理 (卡住检测、重规划)
"""
import math
import time
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import Enum


class NavState(Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    ARRIVED = "ARRIVED"
    STUCK = "STUCK"
    REPLAN = "REPLAN"


@dataclass
class NavResult:
    state: NavState
    global_path: List[Tuple[float, float]] = field(default_factory=list)
    local_path: List[Tuple[float, float]] = field(default_factory=list)
    target_waypoint: int = 0
    total_distance: float = 0.0
    elapsed_time: float = 0.0
    replan_count: int = 0


class WaypointNavigator:
    """航点导航控制器"""

    def __init__(self, planner, local_planner, robot, resolution=0.05):
        self.planner = planner          # A* 全局规划器
        self.local_planner = local_planner  # DWA 局部规划器
        self.robot = robot
        self.resolution = resolution

        # 状态
        self.state = NavState.IDLE
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_idx = 0
        self.global_path: List[Tuple[float, float]] = []
        self.local_path: List[Tuple[float, float]] = []
        self.path_index = 0

        # 统计
        self.total_distance = 0.0
        self.start_time = 0.0
        self.replan_count = 0
        self.collision_count = 0
        self.last_positions: List[Tuple[float, float]] = []

        # 参数
        self.goal_tolerance = 0.3      # 到达容差 (m)
        self.waypoint_tolerance = 0.5
        self.stuck_threshold = 0.05    # 最小移动距离 (m)
        self.stuck_time = 2.0          # 卡住判定时间 (s)
        self.replan_interval = 3.0     # 重规划间隔 (s)
        self.last_replan_time = 0.0

        # 扫描缓存
        self.last_scan = []
        self.scan_hz = 0
        self._scan_times = []

    def set_waypoints(self, waypoints: List[Tuple[float, float]]):
        """设置航点序列"""
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.state = NavState.IDLE
        self.total_distance = 0.0
        self.replan_count = 0
        self.collision_count = 0
        self.last_positions = []

    def start(self):
        """开始导航"""
        if not self.waypoints:
            return
        self.state = NavState.PLANNING
        self.start_time = time.time()

    def update(self, dt: float) -> NavResult:
        """主更新循环"""
        # 获取传感器数据 (外部会注入)
        scan = self.robot.get_laser_scan(
            self.resolution
        )

        result = NavResult(state=self.state)

        if self.state == NavState.IDLE:
            return result

        current_pos = (self.robot.state.x, self.robot.state.y)

        if self.state == NavState.PLANNING:
            self._plan_to_current_waypoint()
            result.state = self.state
            result.global_path = self.global_path
            return result

        elif self.state == NavState.NAVIGATING:
            # 跟踪目标
            target = self._get_current_target(current_pos)
            if target is None:
                # 当前航点到达
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx >= len(self.waypoints):
                    self.state = NavState.ARRIVED
                    result.state = self.state
                    result.elapsed_time = time.time() - self.start_time
                    result.total_distance = self.total_distance
                    return result
                else:
                    self.state = NavState.PLANNING
                    result.state = self.state
                    return result

            # DWA 局部规划
            goal_pose = type('Pose', (), {'x': target[0], 'y': target[1], 'theta': 0.0})()
            current_pose = type('Pose', (), {
                'x': self.robot.state.x,
                'y': self.robot.state.y,
                'theta': self.robot.state.theta
            })()
            current_vel = type('Velocity', (), {
                'v': self.robot.state.v,
                'w': self.robot.state.w
            })()

            vel_cmd, local_traj = self.local_planner.plan(current_pose, goal_pose, current_vel)

            # 执行速度指令
            self.robot.update(vel_cmd.v, vel_cmd.w, dt)

            # 轨迹跟踪
            if local_traj:
                self.local_path = [(p.x, p.y) for p in local_traj]

            # 距离统计
            if self.last_positions:
                last = self.last_positions[-1]
                dist = math.sqrt(
                    (current_pos[0] - last[0]) ** 2 +
                    (current_pos[1] - last[1]) ** 2
                )
                self.total_distance += dist
            self.last_positions.append(current_pos)

            # 卡住检测
            if self._check_stuck():
                self.state = NavState.REPLAN
                self.replan_count += 1
                result.state = self.state
                return result

            # 碰撞检测
            if self.robot.check_collision(
                    np.zeros((1, 1), dtype=bool),
                    self.resolution):
                self.collision_count += 1

            # 定期重规划
            if time.time() - self.last_replan_time > self.replan_interval:
                if self._needs_replan():
                    self.state = NavState.REPLAN
                    self.replan_count += 1

            result.state = self.state
            result.global_path = self.global_path
            result.local_path = self.local_path
            result.target_waypoint = self.current_waypoint_idx
            result.total_distance = self.total_distance
            result.elapsed_time = time.time() - self.start_time
            result.replan_count = self.replan_count

            return result

        elif self.state == NavState.REPLAN:
            self._plan_to_current_waypoint()
            result.state = self.state
            result.global_path = self.global_path
            return result

        elif self.state == NavState.ARRIVED:
            result.state = self.state
            result.elapsed_time = time.time() - self.start_time
            result.total_distance = self.total_distance
            return result

        return result

    def _plan_to_current_waypoint(self):
        """规划到当前航点"""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.state = NavState.ARRIVED
            return

        target = self.waypoints[self.current_waypoint_idx]
        start = (
            int(self.robot.state.x / self.resolution),
            int(self.robot.state.y / self.resolution)
        )
        goal = (
            int(target[0] / self.resolution),
            int(target[1] / self.resolution)
        )

        t0 = time.time()
        path = self.planner.plan(start, goal)
        plan_time = time.time() - t0

        if path:
            self.global_path = [(x * self.resolution, y * self.resolution) for x, y in path]
            self.path_index = 0
            self.state = NavState.NAVIGATING
            self.last_replan_time = time.time()
        else:
            # 跳过这个航点
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.state = NavState.ARRIVED
            else:
                self.state = NavState.PLANNING

    def _get_current_target(self, current_pos: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """获取当前局部目标"""
        if not self.global_path or self.path_index >= len(self.global_path):
            return None

        # 沿路径前进
        target = self.global_path[self.path_index]
        dist = math.sqrt(
            (current_pos[0] - target[0]) ** 2 +
            (current_pos[1] - target[1]) ** 2
        )

        if dist < self.goal_tolerance:
            self.path_index += 1
            if self.path_index >= len(self.global_path):
                return None

        # 前瞻: 找到前方几个点作为局部目标
        lookahead = min(self.path_index + 10, len(self.global_path) - 1)
        return self.global_path[lookahead]

    def _check_stuck(self) -> bool:
        """检测是否卡住"""
        if len(self.last_positions) < 20:
            return False

        recent = self.last_positions[-20:]
        total_move = 0
        for i in range(1, len(recent)):
            total_move += math.sqrt(
                (recent[i][0] - recent[i - 1][0]) ** 2 +
                (recent[i][1] - recent[i - 1][1]) ** 2
            )

        return total_move < self.stuck_threshold

    def _needs_replan(self) -> bool:
        """判断是否需要重规划"""
        if not self.global_path:
            return True
        return False

    def get_scan_hz(self) -> float:
        return self.scan_hz
