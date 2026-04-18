"""
路径规划模块
- A* 全局规划
- D* Lite 增量规划
- RRT* 采样规划
- DWA 局部路径规划 (动态窗口法)
"""
import numpy as np
import heapq
import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Set
import time


@dataclass
class Pose:
    x: float
    y: float
    theta: float = 0.0


@dataclass
class Velocity:
    v: float = 0.0    # linear
    w: float = 0.0    # angular


class AStarPlanner:
    """A* 全局路径规划器"""

    def __init__(self, costmap: np.ndarray, resolution: float = 0.05):
        self.costmap = costmap
        self.resolution = resolution
        self.height, self.width = costmap.shape
        # 8-connected neighbors
        self.directions = [
            (0, 1), (0, -1), (1, 0), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int],
             cost_weight: float = 1.0, iteration_limit: int = 50000) -> Optional[List[Tuple[int, int]]]:
        sx, sy = start
        gx, gy = goal

        if not self._in_bounds(sx, sy) or not self._in_bounds(gx, gy):
            return None

        open_set = []
        heapq.heappush(open_set, (0.0, sx, sy))
        came_from = {}
        g_score = {(sx, sy): 0.0}
        closed = set()
        iterations = 0

        while open_set and iterations < iteration_limit:
            iterations += 1
            _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) == (gx, gy):
                return self._reconstruct_path(came_from, (cx, cy))

            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))

            for dx, dy in self.directions:
                nx, ny = cx + dx, cy + dy

                if not self._in_bounds(nx, ny) or (nx, ny) in closed:
                    continue

                cost = self.costmap[ny, nx]
                if cost > 200:  # 高代价区域
                    continue

                move_cost = math.sqrt(dx * dx + dy * dy)
                # 加入代价地图惩罚
                move_cost += cost * cost_weight * 0.01

                tentative_g = g_score[(cx, cy)] + move_cost

                if tentative_g < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + self._heuristic(nx, ny, gx, gy)
                    heapq.heappush(open_set, (f, nx, ny))

        return None  # 未找到路径

    def _in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def _heuristic(self, x1, y1, x2, y2) -> float:
        # Octile distance
        dx = abs(x1 - x2)
        dy = abs(y1 - y2)
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


class RRTStarPlanner:
    """RRT* 采样路径规划器 — 适合复杂环境"""

    def __init__(self, obstacle_mask: np.ndarray, resolution: float = 0.05):
        self.obstacle_mask = obstacle_mask
        self.resolution = resolution
        self.height, self.width = obstacle_mask.shape
        self.max_iter = 3000
        self.step_size = 5.0  # pixels
        self.goal_bias = 0.15
        self.rewire_radius = 15.0

    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        nodes = [start]
        parents = {0: -1}
        costs = {0: 0.0}
        goal_node = None

        for i in range(1, self.max_iter):
            # Goal bias sampling
            if np.random.random() < self.goal_bias:
                sample = goal
            else:
                sample = (np.random.uniform(0, self.width),
                          np.random.uniform(0, self.height))

            # Find nearest node
            nearest_idx = self._nearest(nodes, sample)
            nearest = nodes[nearest_idx]

            # Steer towards sample
            new_node = self._steer(nearest, sample, self.step_size)

            if not self._is_valid(nearest, new_node):
                continue

            # Find neighbors for rewiring
            neighbors = self._find_neighbors(nodes, new_node, self.rewire_radius)

            # Choose best parent
            best_parent = nearest_idx
            best_cost = costs[nearest_idx] + self._dist(nearest, new_node)

            for n_idx in neighbors:
                n_cost = costs[n_idx] + self._dist(nodes[n_idx], new_node)
                if n_cost < best_cost and self._is_valid(nodes[n_idx], new_node):
                    best_parent = n_idx
                    best_cost = n_cost

            new_idx = len(nodes)
            nodes.append(new_node)
            parents[new_idx] = best_parent
            costs[new_idx] = best_cost

            # Rewire neighbors
            for n_idx in neighbors:
                new_cost = best_cost + self._dist(new_node, nodes[n_idx])
                if new_cost < costs[n_idx] and self._is_valid(new_node, nodes[n_idx]):
                    parents[n_idx] = new_idx
                    costs[n_idx] = new_cost

            # Check if goal reached
            if self._dist(new_node, goal) < self.step_size * 1.5:
                if goal_node is None or best_cost < costs[goal_node]:
                    goal_node = new_idx

        if goal_node is None:
            return None

        # Reconstruct path
        path = []
        idx = goal_node
        while idx != -1:
            path.append(nodes[idx])
            idx = parents[idx]
        path.reverse()

        # Smooth the path
        path = self._smooth(path)
        return path

    def _nearest(self, nodes, point):
        dists = [self._dist(n, point) for n in nodes]
        return int(np.argmin(dists))

    def _find_neighbors(self, nodes, point, radius):
        return [i for i, n in enumerate(nodes) if self._dist(n, point) < radius]

    def _steer(self, from_pt, to_pt, max_dist):
        dx = to_pt[0] - from_pt[0]
        dy = to_pt[1] - from_pt[1]
        d = math.sqrt(dx * dx + dy * dy)
        if d < max_dist:
            return to_pt
        ratio = max_dist / d
        return (from_pt[0] + dx * ratio, from_pt[1] + dy * ratio)

    def _is_valid(self, p1, p2) -> bool:
        steps = max(int(self._dist(p1, p2) / 2), 1)
        for i in range(steps + 1):
            t = i / steps
            x = int(p1[0] + t * (p2[0] - p1[0]))
            y = int(p1[1] + t * (p2[1] - p1[1]))
            if 0 <= x < self.width and 0 <= y < self.height:
                if self.obstacle_mask[y, x]:
                    return False
            else:
                return False
        return True

    def _dist(self, p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def _smooth(self, path, iterations=50):
        if len(path) <= 2:
            return path
        for _ in range(iterations):
            if len(path) <= 2:
                break
            i = np.random.randint(0, len(path) - 2)
            j = np.random.randint(i + 2, len(path))
            if self._is_valid(path[i], path[j]):
                path = path[:i + 1] + path[j:]
        return path


class DWAPlanner:
    """动态窗口法 (DWA) 局部路径规划器"""

    def __init__(self, costmap: np.ndarray, resolution: float = 0.05):
        self.costmap = costmap
        self.resolution = resolution
        self.height, self.width = costmap.shape

        # 机器人参数
        self.max_speed = 1.0       # m/s
        self.min_speed = -0.2
        self.max_yaw_rate = 2.0    # rad/s
        self.max_accel = 0.5       # m/s^2
        self.max_dyaw_rate = 3.0   # rad/s^2
        self.v_resolution = 0.05
        self.w_resolution = 0.05
        self.dt = 0.1              # 预测时间步
        self.predict_time = 1.5    # 预测时长
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 0.5
        self.obstacle_cost_gain = 2.0
        self.robot_radius = 0.3    # meters

    def plan(self, state: Pose, goal: Pose, current_vel: Velocity) -> Velocity:
        """计算最优速度指令"""
        dw = self._calc_dynamic_window(current_vel)
        best_vel = Velocity(0, 0)
        best_cost = -float('inf')
        best_trajectory = None

        v = dw['v_min']
        while v <= dw['v_max']:
            w = dw['w_min']
            while w <= dw['w_max']:
                trajectory = self._predict_trajectory(state, v, w)
                to_goal_cost = self._calc_to_goal_cost(trajectory, goal)
                speed_cost = v / self.max_speed
                ob_cost = self._calc_obstacle_cost(trajectory)

                final_cost = (self.to_goal_cost_gain * to_goal_cost +
                              self.speed_cost_gain * speed_cost -
                              self.obstacle_cost_gain * ob_cost)

                if final_cost > best_cost:
                    best_cost = final_cost
                    best_vel = Velocity(v, w)
                    best_trajectory = trajectory

                w += self.w_resolution
            v += self.v_resolution

        return best_vel, best_trajectory

    def _calc_dynamic_window(self, vel: Velocity) -> dict:
        vs = {
            'v_min': max(self.min_speed, vel.v - self.max_accel * self.dt),
            'v_max': min(self.max_speed, vel.v + self.max_accel * self.dt),
            'w_min': max(-self.max_yaw_rate, vel.w - self.max_dyaw_rate * self.dt),
            'w_max': min(self.max_yaw_rate, vel.w + self.max_dyaw_rate * self.dt),
        }
        return vs

    def _predict_trajectory(self, state: Pose, v: float, w: float) -> List[Pose]:
        trajectory = []
        x, y, theta = state.x, state.y, state.theta
        time = 0.0
        while time <= self.predict_time:
            trajectory.append(Pose(x, y, theta))
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            time += self.dt
        return trajectory

    def _calc_to_goal_cost(self, trajectory: List[Pose], goal: Pose) -> float:
        if not trajectory:
            return 0.0
        last = trajectory[-1]
        dx = goal.x - last.x
        dy = goal.y - last.y
        dist = math.sqrt(dx * dx + dy * dy)
        return 1.0 / (dist + 0.1)

    def _calc_obstacle_cost(self, trajectory: List[Pose]) -> float:
        min_dist = float('inf')
        robot_radius_px = int(self.robot_radius / self.resolution)

        for pose in trajectory:
            px = int(pose.x / self.resolution)
            py = int(pose.y / self.resolution)

            # 检查周围区域
            for dx in range(-robot_radius_px, robot_radius_px + 1, 2):
                for dy in range(-robot_radius_px, robot_radius_px + 1, 2):
                    cx, cy = px + dx, py + dy
                    if 0 <= cx < self.width and 0 <= cy < self.height:
                        if self.costmap[cy, cx] > 100:
                            d = math.sqrt(dx * dx + dy * dy) * self.resolution
                            min_dist = min(min_dist, d)

        if min_dist == float('inf'):
            return 1.0
        return min(min_dist / (self.robot_radius * 2), 1.0)


def path_to_world(path: List[Tuple[int, int]], resolution: float) -> List[Tuple[float, float]]:
    """栅格坐标 → 世界坐标"""
    return [(x * resolution, y * resolution) for x, y in path]


def smooth_path(path: List[Tuple[float, float]], weight_smooth=0.3, weight_data=0.5,
                tolerance=0.0001) -> List[Tuple[float, float]]:
    """梯度下降路径平滑"""
    if len(path) <= 2:
        return path

    new_path = [list(p) for p in path]
    change = tolerance

    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path) - 1):
            for j in range(2):  # x, y
                aux = new_path[i][j]
                new_path[i][j] += weight_data * (path[i][j] - new_path[i][j])
                new_path[i][j] += weight_smooth * (
                    new_path[i - 1][j] + new_path[i + 1][j] - 2 * new_path[i][j]
                )
                change += abs(aux - new_path[i][j])

    return [tuple(p) for p in new_path]
