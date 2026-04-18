"""
主可视化引擎 — Pygame 渲染
- 地图渲染 (分层着色)
- 机器人可视化 (含动画)
- 路径可视化 (发光效果)
- 激光雷达可视化
- 代价地图叠加
- HUD 遥测面板
- 粒子效果
"""
import pygame
import numpy as np
import math
import time
import colorsys
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
from collections import deque


# ─── 配色方案 ─────────────────────────────────────────────
class Colors:
    BG = (12, 12, 22)
    GRID = (25, 25, 40)

    # 地图元素 — 工业风配色
    FREE = (16, 16, 22)
    WALL = (55, 60, 70)
    WALL_HIGHLIGHT = (70, 75, 90)
    FURNITURE = (80, 60, 40)
    PILLAR = (75, 80, 95)
    PILLAR_INNER = (60, 65, 75)
    DOOR = (50, 120, 80)
    SHELF = (90, 75, 50)
    PLANT = (30, 100, 50)
    VENT = (25, 25, 40)
    # 工厂特有
    CONVEYOR = (120, 100, 30)       # 传送带 — 暗金
    CONVEYOR_BELT = (80, 70, 20)    # 传送带内部
    MACHINE = (100, 55, 45)         # 机器 — 暗红
    MACHINE_HIGHLIGHT = (140, 70, 55)
    PIPE = (60, 80, 100)            # 管道 — 钢蓝
    MARK = (40, 100, 60)            # 地面标线 — 绿
    LOADING = (80, 65, 45)          # 装卸区 — 棕
    CHARGER = (30, 130, 100)        # 充电站 — 青绿
    SAFETY = (120, 90, 20)          # 安全围栏 — 警示黄

    # 区域色调 — 工厂分区
    ZONE_COLORS = [
        (16, 16, 22),      # 无区域
        (22, 16, 18),      # A 原材料仓库 — 微红
        (18, 22, 22),      # B 生产线1 — 微青
        (16, 20, 25),      # C 生产线2 — 微蓝
        (22, 20, 16),      # D 质检区 — 微黄
        (18, 16, 22),      # E 成品仓库 — 微紫
        (22, 18, 16),      # F 维修间 — 微橙
        (20, 20, 20),      # G 主干道 — 灰
        (16, 20, 18),      # H 装卸码头 — 微绿
    ]

    # 路径
    PATH_GLOBAL = (255, 100, 180)
    PATH_LOCAL = (100, 200, 255)
    PATH_TRAIL = (120, 80, 200)

    # 机器人
    ROBOT_BODY = (60, 180, 255)
    ROBOT_HEAD = (255, 200, 50)
    ROBOT_SHADOW = (20, 60, 100)

    # 传感器
    LASER_RAY = (255, 50, 50, 60)
    LASER_POINT = (255, 100, 100)

    # HUD
    HUD_BG = (8, 8, 18, 200)
    HUD_TEXT = (180, 200, 220)
    HUD_ACCENT = (60, 180, 255)
    HUD_WARN = (255, 180, 50)
    HUD_DANGER = (255, 60, 60)
    HUD_OK = (50, 200, 100)

    # 代价地图
    COSTMAP_LOW = (0, 255, 0)
    COSTMAP_MED = (255, 255, 0)
    COSTMAP_HIGH = (255, 50, 0)

    # 目标点
    GOAL = (255, 50, 180)
    GOAL_PULSE = (255, 100, 200, 80)
    WAYPOINT = (200, 150, 255)
    WAYPOINT_DONE = (80, 200, 80)


class ParticleSystem:
    """粒子系统 — 烟雾、火花等效果"""

    @dataclass
    class Particle:
        x: float
        y: float
        vx: float
        vy: float
        life: float
        max_life: float
        size: float
        color: Tuple[int, int, int]
        alpha: int = 255

    def __init__(self):
        self.particles: List['ParticleSystem.Particle'] = []
        self.max_particles = 500

    def emit(self, x: float, y: float, count: int = 5,
             color=(255, 200, 100), spread=2.0, speed=30.0,
             lifetime=0.8, size=2.0):
        for _ in range(count):
            if len(self.particles) >= self.max_particles:
                break
            angle = np.random.uniform(0, 2 * math.pi)
            spd = np.random.uniform(speed * 0.5, speed)
            self.particles.append(self.Particle(
                x=x, y=y,
                vx=math.cos(angle) * spd,
                vy=math.sin(angle) * spd,
                life=lifetime * np.random.uniform(0.5, 1.0),
                max_life=lifetime,
                size=size * np.random.uniform(0.5, 1.5),
                color=color
            ))

    def emit_trail(self, x: float, y: float, color=(100, 150, 255)):
        """机器人运动轨迹粒子"""
        self.emit(x, y, count=1, color=color, spread=1.0,
                  speed=5.0, lifetime=0.5, size=1.5)

    def update(self, dt: float):
        alive = []
        for p in self.particles:
            p.life -= dt
            if p.life > 0:
                p.x += p.vx * dt
                p.y += p.vy * dt
                p.vx *= 0.95
                p.vy *= 0.95
                p.alpha = int(255 * (p.life / p.max_life))
                alive.append(p)
        self.particles = alive

    def draw(self, surface: pygame.Surface, offset_x: float, offset_y: float, scale: float):
        for p in self.particles:
            sx = int(p.x * scale + offset_x)
            sy = int(p.y * scale + offset_y)
            if 0 <= sx < surface.get_width() and 0 <= sy < surface.get_height():
                size = max(1, int(p.size * scale / 10))
                color = (*p.color[:3], min(255, p.alpha))
                try:
                    pygame.draw.circle(surface, p.color, (sx, sy), size)
                except:
                    pass


class SensorVisualizer:
    """传感器数据可视化"""

    def __init__(self):
        self.laser_surface: Optional[pygame.Surface] = None
        self.scan_time_display = deque(maxlen=60)

    def draw_laser_scan(self, surface: pygame.Surface, scan_data: List[float],
                        robot_x: float, robot_y: float, robot_theta: float,
                        offset_x: float, offset_y: float, scale: float):
        """绘制激光雷达扫描"""
        px = robot_x * scale + offset_x
        py = robot_y * scale + offset_y

        for i, dist in enumerate(scan_data):
            angle = robot_theta + (i / len(scan_data)) * 2 * math.pi
            if dist < 9.5:  # 有效范围
                ex = px + dist * math.cos(angle) * scale
                ey = py + dist * math.sin(angle) * scale

                # 射线
                alpha = max(20, 80 - int(dist * 8))
                color = (255, 50, 50, alpha)
                try:
                    pygame.draw.line(surface, (255, 50, 50),
                                     (int(px), int(py)), (int(ex), int(ey)), 1)
                    # 点
                    pygame.draw.circle(surface, (255, 100, 100),
                                       (int(ex), int(ey)), 2)
                except:
                    pass


class NavigationVisualizer:
    """主可视化引擎"""

    def __init__(self, width=1600, height=900):
        pygame.init()
        self.screen_width = width
        self.screen_height = height
        self.screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
        pygame.display.set_caption("🤖 ROS 2 Navigation Simulator [Nav2 Stack]")

        # 渲染层
        self.map_surface = pygame.Surface((width, height))
        self.overlay_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        self.hud_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        self.costmap_surface = pygame.Surface((width, height), pygame.SRCALPHA)

        # 粒子系统
        self.particles = ParticleSystem()

        # 传感器可视化
        self.sensor_viz = SensorVisualizer()

        # 视图控制
        self.view_offset_x = 0.0
        self.view_offset_y = 0.0
        self.view_scale = 4.0
        self.target_scale = 4.0
        self.is_dragging = False
        self.drag_start = (0, 0)

        # 动画状态
        self.time = 0.0
        self.frame_count = 0
        self.fps_history = deque(maxlen=60)
        self.show_costmap = True
        self.show_laser = True
        self.show_trajectory = True
        self.show_grid = False

        # 字体
        try:
            self.font_sm = pygame.font.SysFont("monospace", 12)
            self.font_md = pygame.font.SysFont("monospace", 14)
            self.font_lg = pygame.font.SysFont("monospace", 18, bold=True)
            self.font_title = pygame.font.SysFont("monospace", 22, bold=True)
        except:
            self.font_sm = pygame.font.Font(None, 16)
            self.font_md = pygame.font.Font(None, 18)
            self.font_lg = pygame.font.Font(None, 22)
            self.font_title = pygame.font.Font(None, 26)

        # 地图数据
        self.map_data = None
        self.map_rendered = False
        self.map_cache = None

        # 状态
        self.global_path = []
        self.local_path = []
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.nav_status = "IDLE"
        self.planning_algorithm = "A*"

        # 遥测数据
        self.telemetry = {
            'v': 0.0, 'w': 0.0, 'x': 0.0, 'y': 0.0, 'theta': 0.0,
            'plan_time': 0.0, 'total_dist': 0.0, 'waypoints_done': 0,
            'collisions': 0, 'replans': 0,
        }

        # 日志
        self.log_messages = deque(maxlen=8)

    def load_map(self, map_data: dict):
        """加载并预渲染地图"""
        self.map_data = map_data
        self._prerender_map()
        self.map_rendered = True

        # 自动缩放适配
        map_w = map_data['width'] * map_data.get('resolution', 0.05)
        map_h = map_data['height'] * map_data.get('resolution', 0.05)
        scale_x = (self.screen_width - 350) / map_w
        scale_y = (self.screen_height - 100) / map_h
        self.view_scale = min(scale_x, scale_y) * 0.8
        self.target_scale = self.view_scale

        # 居中
        self.view_offset_x = 50
        self.view_offset_y = 30

    def _prerender_map(self):
        """预渲染静态地图"""
        grid = self.map_data['grid']
        zone_map = self.map_data['zone_map']
        h, w = grid.shape

        surface = pygame.Surface((w, h))

        # 先绘制区域底色
        zone_pixels = pygame.surfarray.pixels3d(surface)
        for y in range(h):
            for x in range(w):
                zone_id = zone_map[y, x]
                if zone_id < len(Colors.ZONE_COLORS):
                    zone_pixels[x][y] = Colors.ZONE_COLORS[zone_id]
                else:
                    zone_pixels[x][y] = Colors.FREE
        del zone_pixels

        # 绘制特殊元素
        for y in range(h):
            for x in range(w):
                cell = grid[y, x]
                if cell == 1:     # WALL
                    surface.set_at((x, y), Colors.WALL)
                elif cell == 2:   # PILLAR
                    surface.set_at((x, y), Colors.PILLAR)
                elif cell == 3:   # CONVEYOR
                    surface.set_at((x, y), Colors.CONVEYOR)
                elif cell == 4:   # MACHINE
                    surface.set_at((x, y), Colors.MACHINE)
                elif cell == 5:   # SHELF
                    surface.set_at((x, y), Colors.SHELF)
                elif cell == 6:   # PIPE
                    surface.set_at((x, y), Colors.PIPE)
                elif cell == 7:   # VENT
                    surface.set_at((x, y), Colors.VENT)
                elif cell == 8:   # MARK (地面标线)
                    surface.set_at((x, y), Colors.MARK)
                elif cell == 9:   # LOADING
                    surface.set_at((x, y), Colors.LOADING)
                elif cell == 10:  # CHARGER
                    surface.set_at((x, y), Colors.CHARGER)
                elif cell == 11:  # SAFETY
                    surface.set_at((x, y), Colors.SAFETY)

        self.map_cache = surface

    def _render_costmap_overlay(self):
        """渲染代价地图叠加层"""
        if self.map_data is None:
            return

        costmap = self.map_data['costmap']
        h, w = costmap.shape

        self.costmap_surface.fill((0, 0, 0, 0))

        for y in range(0, h, 2):
            for x in range(0, w, 2):
                cost = costmap[y, x]
                if cost > 10:
                    # 从绿到红的渐变
                    ratio = min(cost / 255, 1.0)
                    if ratio < 0.5:
                        r = int(255 * ratio * 2)
                        g = 255
                        b = 0
                    else:
                        r = 255
                        g = int(255 * (1 - ratio) * 2)
                        b = 0
                    alpha = int(min(cost * 0.3, 80))

                    sx = x * self.view_scale + self.view_offset_x
                    sy = y * self.view_scale + self.view_offset_y
                    size = max(1, int(2 * self.view_scale))
                    try:
                        pygame.draw.rect(self.costmap_surface, (r, g, b, alpha),
                                         (int(sx), int(sy), size, size))
                    except:
                        pass

    def draw_map(self):
        """绘制地图"""
        if self.map_cache is None:
            return

        scaled_w = int(self.map_cache.get_width() * self.view_scale)
        scaled_h = int(self.map_cache.get_height() * self.view_scale)

        if scaled_w > 0 and scaled_h > 0:
            scaled_map = pygame.transform.scale(self.map_cache, (scaled_w, scaled_h))
            self.screen.blit(scaled_map, (int(self.view_offset_x), int(self.view_offset_y)))

    def draw_costmap(self):
        """绘制代价地图"""
        if self.show_costmap:
            self._render_costmap_overlay()
            self.screen.blit(self.costmap_surface, (0, 0))

    def draw_robot(self, x: float, y: float, theta: float):
        """绘制机器人 (带发光效果)"""
        px = x * self.view_scale + self.view_offset_x
        py = y * self.view_scale + self.view_offset_y
        radius = max(4, int(0.3 * self.view_scale))  # 30cm 机器人半径

        # 发光效果
        glow_radius = radius * 3
        glow_surface = pygame.Surface((glow_radius * 2, glow_radius * 2), pygame.SRCALPHA)
        for r in range(glow_radius, 0, -1):
            alpha = int(30 * (r / glow_radius))
            color = (60, 180, 255, alpha)
            try:
                pygame.draw.circle(glow_surface, color,
                                   (glow_radius, glow_radius), r)
            except:
                pass
        self.screen.blit(glow_surface,
                         (int(px - glow_radius), int(py - glow_radius)))

        # 阴影
        pygame.draw.circle(self.screen, Colors.ROBOT_SHADOW,
                           (int(px + 2), int(py + 2)), radius)

        # 机身
        pygame.draw.circle(self.screen, Colors.ROBOT_BODY,
                           (int(px), int(py)), radius)
        pygame.draw.circle(self.screen, (100, 210, 255),
                           (int(px), int(py)), radius, 2)

        # 方向指示 (朝向)
        head_x = px + radius * 1.3 * math.cos(theta)
        head_y = py + radius * 1.3 * math.sin(theta)
        pygame.draw.circle(self.screen, Colors.ROBOT_HEAD,
                           (int(head_x), int(head_y)), max(2, radius // 3))

        # 连线
        pygame.draw.line(self.screen, Colors.ROBOT_HEAD,
                         (int(px), int(py)), (int(head_x), int(head_y)), 2)

    def draw_trajectory(self, trajectory: List[Tuple[float, float, float]]):
        """绘制历史轨迹 (渐变色)"""
        if not self.show_trajectory or len(trajectory) < 2:
            return

        n = len(trajectory)
        for i in range(1, n):
            alpha = int(255 * (i / n))
            ratio = i / n

            # 从紫到蓝的渐变
            r = int(120 + 135 * ratio)
            g = int(60 + 120 * ratio)
            b = int(200 + 55 * ratio)

            x1 = trajectory[i - 1][0] * self.view_scale + self.view_offset_x
            y1 = trajectory[i - 1][1] * self.view_scale + self.view_offset_y
            x2 = trajectory[i][0] * self.view_scale + self.view_offset_x
            y2 = trajectory[i][1] * self.view_scale + self.view_offset_y

            try:
                pygame.draw.line(self.screen, (r, g, b),
                                 (int(x1), int(y1)), (int(x2), int(y2)), 2)
            except:
                pass

        # 尾部粒子
        if n > 0 and self.frame_count % 3 == 0:
            last = trajectory[-1]
            self.particles.emit_trail(
                last[0], last[1],
                color=(150, 100, 255)
            )

    def draw_global_path(self, path: List[Tuple[float, float]]):
        """绘制全局路径 (发光脉冲)"""
        if len(path) < 2:
            return

        # 脉冲效果
        pulse = (math.sin(self.time * 4) + 1) / 2

        for i in range(1, len(path)):
            x1 = path[i - 1][0] * self.view_scale + self.view_offset_x
            y1 = path[i - 1][1] * self.view_scale + self.view_offset_y
            x2 = path[i][0] * self.view_scale + self.view_offset_x
            y2 = path[i][1] * self.view_scale + self.view_offset_y

            # 发光层
            for offset in range(3, 0, -1):
                alpha = int(30 * pulse / offset)
                try:
                    pygame.draw.line(self.screen, (255, 100, 180),
                                     (int(x1), int(y1)), (int(x2), int(y2)),
                                     offset * 2 + 1)
                except:
                    pass

            # 主线
            try:
                pygame.draw.line(self.screen, Colors.PATH_GLOBAL,
                                 (int(x1), int(y1)), (int(x2), int(y2)), 2)
            except:
                pass

            # 方向箭头 (每隔N个点)
            if i % 8 == 0:
                angle = math.atan2(y2 - y1, x2 - x1)
                arrow_len = 6
                ax = x2 - arrow_len * math.cos(angle - 0.4)
                ay = y2 - arrow_len * math.sin(angle - 0.4)
                bx = x2 - arrow_len * math.cos(angle + 0.4)
                by = y2 - arrow_len * math.sin(angle + 0.4)
                try:
                    pygame.draw.polygon(self.screen, Colors.PATH_GLOBAL,
                                        [(int(x2), int(y2)), (int(ax), int(ay)),
                                         (int(bx), int(by))])
                except:
                    pass

    def draw_local_path(self, path: List[Tuple[float, float]]):
        """绘制局部路径"""
        if len(path) < 2:
            return

        for i in range(1, len(path)):
            x1 = path[i - 1][0] * self.view_scale + self.view_offset_x
            y1 = path[i - 1][1] * self.view_scale + self.view_offset_y
            x2 = path[i][0] * self.view_scale + self.view_offset_x
            y2 = path[i][1] * self.view_scale + self.view_offset_y

            try:
                pygame.draw.line(self.screen, Colors.PATH_LOCAL,
                                 (int(x1), int(y1)), (int(x2), int(y2)), 3)
            except:
                pass

    def draw_waypoints(self, waypoints: List[Tuple[float, float]],
                       current_idx: int):
        """绘制航点"""
        for i, (wx, wy) in enumerate(waypoints):
            px = wx * self.view_scale + self.view_offset_x
            py = wy * self.view_scale + self.view_offset_y

            if i < current_idx:
                color = Colors.WAYPOINT_DONE
                # 完成标记
                pygame.draw.circle(self.screen, color, (int(px), int(py)), 6)
                pygame.draw.circle(self.screen, (255, 255, 255),
                                   (int(px), int(py)), 6, 1)
            elif i == current_idx:
                # 当前目标 — 脉冲效果
                pulse = (math.sin(self.time * 6) + 1) / 2
                glow_r = int(15 + 10 * pulse)
                glow_surface = pygame.Surface((glow_r * 2, glow_r * 2), pygame.SRCALPHA)
                for r in range(glow_r, 0, -1):
                    alpha = int(40 * (r / glow_r) * pulse)
                    try:
                        pygame.draw.circle(glow_surface, (*Colors.GOAL[:3], alpha),
                                           (glow_r, glow_r), r)
                    except:
                        pass
                self.screen.blit(glow_surface,
                                 (int(px - glow_r), int(py - glow_r)))

                pygame.draw.circle(self.screen, Colors.GOAL, (int(px), int(py)), 8)
                pygame.draw.circle(self.screen, (255, 255, 255),
                                   (int(px), int(py)), 8, 2)

                # 编号
                text = self.font_md.render(str(i + 1), True, (255, 255, 255))
                self.screen.blit(text, (int(px) - 5, int(py) - 8))
            else:
                # 未到达
                color = Colors.WAYPOINT
                pygame.draw.circle(self.screen, color, (int(px), int(py)), 5)
                pygame.draw.circle(self.screen, (255, 255, 255),
                                   (int(px), int(py)), 5, 1)
                text = self.font_sm.render(str(i + 1), True, (200, 200, 200))
                self.screen.blit(text, (int(px) - 4, int(py) - 7))

    def draw_laser_scan(self, scan_ranges: List[float],
                        robot_x: float, robot_y: float, robot_theta: float):
        """绘制激光雷达"""
        if not self.show_laser or not scan_ranges:
            return

        px = robot_x * self.view_scale + self.view_offset_x
        py = robot_y * self.view_scale + self.view_offset_y

        for i, dist in enumerate(scan_ranges):
            if dist > 9.5:
                continue

            angle = robot_theta + (i / len(scan_ranges)) * 2 * math.pi
            ex = px + dist * math.cos(angle) * self.view_scale
            ey = py + dist * math.sin(angle) * self.view_scale

            alpha = max(15, 60 - int(dist * 6))
            try:
                pygame.draw.aaline(self.screen, (255, 50, 50),
                                   (int(px), int(py)), (int(ex), int(ey)))
                if i % 3 == 0:
                    pygame.draw.circle(self.screen, (255, 80, 80),
                                       (int(ex), int(ey)), 1)
            except:
                pass

    def draw_hud(self, telemetry: dict):
        """绘制 HUD 面板"""
        hud_w, hud_h = 320, self.screen_height
        hud_x = self.screen_width - hud_w
        hud_y = 0

        # 半透明背景
        hud_bg = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        hud_bg.fill((5, 5, 15, 220))
        self.screen.blit(hud_bg, (hud_x, hud_y))

        # 边框
        pygame.draw.line(self.screen, Colors.HUD_ACCENT,
                         (hud_x, 0), (hud_x, hud_h), 2)

        y_pos = 15
        x_text = hud_x + 15

        # 标题
        title = self.font_title.render("≡ NAVIGATION SYSTEM", True, Colors.HUD_ACCENT)
        self.screen.blit(title, (x_text, y_pos))
        y_pos += 35

        # 状态
        status_color = {
            'NAVIGATING': Colors.HUD_OK,
            'PLANNING': Colors.HUD_WARN,
            'ARRIVED': Colors.HUD_ACCENT,
            'IDLE': Colors.HUD_TEXT,
            'STUCK': Colors.HUD_DANGER,
        }.get(self.nav_status, Colors.HUD_TEXT)

        status_text = self.font_lg.render(f"● {self.nav_status}", True, status_color)
        self.screen.blit(status_text, (x_text, y_pos))
        y_pos += 30

        # 分割线
        pygame.draw.line(self.screen, (40, 40, 60),
                         (x_text, y_pos), (hud_x + hud_w - 15, y_pos))
        y_pos += 15

        # 位姿信息
        sections = [
            ("◆ POSE", [
                ("X", f"{telemetry.get('x', 0):.3f} m"),
                ("Y", f"{telemetry.get('y', 0):.3f} m"),
                ("θ", f"{math.degrees(telemetry.get('theta', 0)):.1f}°"),
            ]),
            ("◆ VELOCITY", [
                ("Linear", f"{telemetry.get('v', 0):.3f} m/s"),
                ("Angular", f"{telemetry.get('w', 0):.3f} rad/s"),
            ]),
            ("◆ NAVIGATION", [
                ("Algorithm", self.planning_algorithm),
                ("Plan Time", f"{telemetry.get('plan_time', 0)*1000:.1f} ms"),
                ("Total Dist", f"{telemetry.get('total_dist', 0):.2f} m"),
                ("Waypoints", f"{telemetry.get('waypoints_done', 0)}/{len(self.waypoints)}"),
                ("Replans", str(telemetry.get('replans', 0))),
                ("Collisions", str(telemetry.get('collisions', 0))),
            ]),
            ("◆ SENSOR", [
                ("Laser", f"{len(telemetry.get('scan_ranges', []))} pts"),
                ("Update", f"{telemetry.get('scan_hz', 0):.0f} Hz"),
            ]),
        ]

        for section_title, items in sections:
            # 区标题
            st = self.font_md.render(section_title, True, Colors.HUD_ACCENT)
            self.screen.blit(st, (x_text, y_pos))
            y_pos += 22

            for label, value in items:
                lt = self.font_sm.render(f"  {label}:", True, Colors.HUD_TEXT)
                vt = self.font_sm.render(value, True, (220, 220, 240))
                self.screen.blit(lt, (x_text, y_pos))
                self.screen.blit(vt, (x_text + 100, y_pos))
                y_pos += 17

            y_pos += 10

        # 速度条
        y_pos += 5
        bar_label = self.font_md.render("◆ SPEED BAR", True, Colors.HUD_ACCENT)
        self.screen.blit(bar_label, (x_text, y_pos))
        y_pos += 22

        v = abs(telemetry.get('v', 0))
        w = abs(telemetry.get('w', 0))
        self._draw_bar(x_text, y_pos, 180, 10, v / 1.5, "V", Colors.HUD_OK)
        y_pos += 18
        self._draw_bar(x_text, y_pos, 180, 10, w / 3.0, "W", Colors.HUD_WARN)
        y_pos += 25

        # 日志
        log_label = self.font_md.render("◆ LOG", True, Colors.HUD_ACCENT)
        self.screen.blit(log_label, (x_text, y_pos))
        y_pos += 20

        for i, msg in enumerate(reversed(self.log_messages)):
            alpha = int(255 * (1 - i / len(self.log_messages))) if self.log_messages else 255
            lt = self.font_sm.render(msg, True, (150, 150, 170))
            self.screen.blit(lt, (x_text, y_pos))
            y_pos += 15

    def _draw_bar(self, x, y, width, height, ratio, label, color):
        """绘制进度条"""
        ratio = max(0, min(1, ratio))

        # 背景
        pygame.draw.rect(self.screen, (30, 30, 45), (x, y, width, height))
        # 前景
        bar_w = int(width * ratio)
        if bar_w > 0:
            pygame.draw.rect(self.screen, color, (x, y, bar_w, height))
        # 边框
        pygame.draw.rect(self.screen, (60, 60, 80), (x, y, width, height), 1)
        # 标签
        lt = self.font_sm.render(f"{label}: {ratio*100:.0f}%", True, (200, 200, 220))
        self.screen.blit(lt, (x + width + 8, y - 2))

    def draw_minimap(self, robot_x, robot_y, robot_theta):
        """右下角小地图"""
        mini_size = 150
        mini_x = self.screen_width - 340 - mini_size
        mini_y = self.screen_height - mini_size - 20

        if self.map_cache is None:
            return

        # 缩放地图
        mini_map = pygame.transform.scale(self.map_cache, (mini_size, mini_size))
        mini_surface = pygame.Surface((mini_size + 4, mini_size + 4), pygame.SRCALPHA)
        mini_surface.fill((0, 0, 0, 180))
        mini_surface.blit(mini_map, (2, 2))

        # 机器人位置
        rx = int(robot_x / self.map_data['width'] * mini_size) + 2
        ry = int(robot_y / self.map_data['height'] * mini_size) + 2
        pygame.draw.circle(mini_surface, Colors.ROBOT_BODY, (rx, ry), 3)

        # 方向
        dx = 5 * math.cos(robot_theta)
        dy = 5 * math.sin(robot_theta)
        pygame.draw.line(mini_surface, Colors.ROBOT_HEAD,
                         (rx, ry), (int(rx + dx), int(ry + dy)), 2)

        # 边框
        pygame.draw.rect(mini_surface, Colors.HUD_ACCENT,
                         (0, 0, mini_size + 4, mini_size + 4), 2)

        self.screen.blit(mini_surface, (mini_x, mini_y))

        # 标签
        label = self.font_sm.render("MINIMAP", True, Colors.HUD_ACCENT)
        self.screen.blit(label, (mini_x, mini_y - 15))

    def draw_title_bar(self):
        """顶部标题栏"""
        bar_h = 35
        bar_surface = pygame.Surface((self.screen_width, bar_h), pygame.SRCALPHA)
        bar_surface.fill((5, 5, 15, 230))
        self.screen.blit(bar_surface, (0, 0))

        # 分割线
        pygame.draw.line(self.screen, Colors.HUD_ACCENT,
                         (0, bar_h), (self.screen_width, bar_h), 1)

        # 标题
        title = self.font_lg.render(
            "🤖 ROS 2 Navigation Simulator  |  Nav2 Stack  |  Topics: /odom /scan /tf /cmd_vel /plan",
            True, Colors.HUD_ACCENT)
        self.screen.blit(title, (20, 8))

        # FPS
        if self.fps_history:
            avg_fps = sum(self.fps_history) / len(self.fps_history)
            fps_text = self.font_md.render(f"FPS: {avg_fps:.0f}", True, Colors.HUD_OK)
            self.screen.blit(fps_text, (self.screen_width - 400, 10))

        # 时间
        time_text = self.font_md.render(
            f"T: {self.time:.1f}s  Frame: {self.frame_count}",
            True, Colors.HUD_TEXT)
        self.screen.blit(time_text, (self.screen_width - 280, 10))

    def draw_controls_help(self):
        """底部控制提示"""
        help_text = (
            "[R] Reset Robot  [C] Toggle Costmap  [L] Toggle Laser  "
            "[G] Toggle Grid  [T] Toggle Trajectory  "
            "[+/-] Zoom  [Drag] Pan  [1-9] Presets  [Space] Pause"
        )
        bar_y = self.screen_height - 25
        bar_surface = pygame.Surface((self.screen_width, 25), pygame.SRCALPHA)
        bar_surface.fill((5, 5, 15, 200))
        self.screen.blit(bar_surface, (0, bar_y))

        ht = self.font_sm.render(help_text, True, (120, 120, 140))
        self.screen.blit(ht, (15, bar_y + 5))

    def add_log(self, msg: str):
        """添加日志"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_messages.append(f"[{timestamp}] {msg}")

    def world_to_screen(self, wx: float, wy: float) -> Tuple[int, int]:
        """世界坐标 → 屏幕坐标"""
        return (int(wx * self.view_scale + self.view_offset_x),
                int(wy * self.view_scale + self.view_offset_y))

    def screen_to_world(self, sx: int, sy: int) -> Tuple[float, float]:
        """屏幕坐标 → 世界坐标"""
        return ((sx - self.view_offset_x) / self.view_scale,
                (sy - self.view_offset_y) / self.view_scale)
