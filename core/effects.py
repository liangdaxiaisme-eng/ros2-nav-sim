"""
视觉特效层 — 叠加在主渲染之上
- CRT 扫描线
- 霓虹发光管线
- 雷达扫描波
- 波形图 (速度/加速度)
- 启动动画
- 碰撞屏幕震动
"""
import pygame
import numpy as np
import math
import time
from typing import List, Tuple, Optional
from collections import deque


class ScanlineEffect:
    """CRT 扫描线效果"""

    def __init__(self, width: int, height: int):
        self.surface = pygame.Surface((width, height), pygame.SRCALPHA)
        for y in range(0, height, 3):
            pygame.draw.line(self.surface, (0, 0, 0, 25),
                             (0, y), (width, y))
        # 边缘暗角
        for i in range(40):
            alpha = int(60 * (1 - i / 40) ** 2)
            overlay = pygame.Surface((width, height), pygame.SRCALPHA)
            pygame.draw.rect(overlay, (0, 0, 0, alpha),
                             (i, i, width - 2 * i, height - 2 * i), 2)
            self.surface.blit(overlay, (0, 0))

    def apply(self, screen: pygame.Surface):
        screen.blit(self.surface, (0, 0))


class NeonGlow:
    """霓虹发光管线绘制"""

    @staticmethod
    def draw_line_glow(surface: pygame.Surface,
                       start: Tuple[int, int], end: Tuple[int, int],
                       color: Tuple[int, int, int],
                       width: int = 2, glow_radius: int = 6,
                       intensity: float = 1.0):
        """带辉光的线"""
        # 多层辉光
        for r in range(glow_radius, 0, -1):
            alpha = int(25 * intensity * (r / glow_radius))
            glow_color = (*color, min(255, alpha))
            try:
                line_surf = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
                pygame.draw.line(line_surf, glow_color, start, end, r * 2)
                surface.blit(line_surf, (0, 0))
            except:
                pass

        # 核心亮线
        try:
            pygame.draw.line(surface, color, start, end, width)
        except:
            pass

    @staticmethod
    def draw_circle_glow(surface: pygame.Surface,
                         center: Tuple[int, int], radius: int,
                         color: Tuple[int, int, int],
                         intensity: float = 1.0):
        """带辉光的圆"""
        for r in range(radius + 6, radius, -1):
            alpha = int(15 * intensity * ((r - radius) / 6))
            try:
                glow_surf = pygame.Surface((r * 2, r * 2), pygame.SRCALPHA)
                pygame.draw.circle(glow_surf, (*color, alpha), (r, r), r)
                surface.blit(glow_surf, (center[0] - r, center[1] - r))
            except:
                pass
        try:
            pygame.draw.circle(surface, color, center, radius)
        except:
            pass


class RadarSweep:
    """雷达扫描波效果"""

    def __init__(self, center_x: int, center_y: int, radius: int):
        self.cx = center_x
        self.cy = center_y
        self.radius = radius
        self.angle = 0.0
        self.trail: deque = deque(maxlen=40)

    def update(self, dt: float, speed: float = 1.5):
        self.angle += speed * dt
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi
        self.trail.append(self.angle)

    def draw(self, surface: pygame.Surface, color=(0, 255, 100)):
        """绘制扫描波"""
        # 尾迹
        for i, angle in enumerate(self.trail):
            alpha = int(40 * (i / len(self.trail)))
            rad = math.radians(angle * 180 / math.pi)
            ex = self.cx + self.radius * math.cos(angle)
            ey = self.cy + self.radius * math.sin(angle)
            try:
                pygame.draw.line(surface, (*color, alpha),
                                 (self.cx, self.cy), (int(ex), int(ey)), 1)
            except:
                pass

        # 当前扫描线
        ex = self.cx + self.radius * math.cos(self.angle)
        ey = self.cy + self.radius * math.sin(self.angle)
        try:
            pygame.draw.line(surface, color,
                             (self.cx, self.cy), (int(ex), int(ey)), 2)
        except:
            pass

        # 中心点
        pygame.draw.circle(surface, color, (self.cx, self.cy), 3)


class WaveformChart:
    """实时波形图"""

    def __init__(self, x: int, y: int, width: int, height: int,
                 label: str, color=(0, 255, 100), max_points: int = 120):
        self.x = x
        self.y = y
        self.w = width
        self.h = height
        self.label = label
        self.color = color
        self.data: deque = deque(maxlen=max_points)
        self.max_val = 1.0
        self.min_val = -1.0

    def add_point(self, value: float):
        self.data.append(value)

    def draw(self, surface: pygame.Surface, time: float):
        """绘制波形"""
        # 背景
        bg = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        bg.fill((0, 0, 0, 120))
        surface.blit(bg, (self.x, self.y))

        # 边框
        pygame.draw.rect(surface, (*self.color[:3], 60),
                         (self.x, self.y, self.w, self.h), 1)

        # 标签
        try:
            font = pygame.font.SysFont("monospace", 10)
        except:
            font = pygame.font.Font(None, 14)
        label = font.render(self.label, True, self.color)
        surface.blit(label, (self.x + 3, self.y + 2))

        # 中线
        mid_y = self.y + self.h // 2
        pygame.draw.line(surface, (*self.color[:3], 30),
                         (self.x, mid_y), (self.x + self.w, mid_y), 1)

        if len(self.data) < 2:
            return

        # 波形线
        points = []
        for i, val in enumerate(self.data):
            px = self.x + int(i / len(self.data) * self.w)
            # 归一化
            if self.max_val != self.min_val:
                norm = (val - self.min_val) / (self.max_val - self.min_val)
            else:
                norm = 0.5
            py = self.y + self.h - int(norm * (self.h - 4)) - 2
            py = max(self.y + 1, min(self.y + self.h - 1, py))
            points.append((px, py))

        if len(points) >= 2:
            # 发光层
            glow_surf = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
            for i in range(1, len(points)):
                try:
                    pygame.draw.line(glow_surf, (*self.color[:3], 40),
                                     points[i - 1], points[i], 4)
                except:
                    pass
            surface.blit(glow_surf, (self.x, self.y))

            try:
                pygame.draw.lines(surface, self.color, False, points, 1)
            except:
                pass

        # 当前值
        if self.data:
            val_text = font.render(f"{self.data[-1]:.2f}", True, (255, 255, 255))
            surface.blit(val_text, (self.x + self.w - 35, self.y + 2))


class ScreenShake:
    """屏幕震动效果"""

    def __init__(self):
        self.intensity = 0.0
        self.duration = 0.0
        self.timer = 0.0
        self.offset_x = 0
        self.offset_y = 0

    def trigger(self, intensity: float = 5.0, duration: float = 0.15):
        self.intensity = intensity
        self.duration = duration
        self.timer = duration

    def update(self, dt: float) -> Tuple[int, int]:
        if self.timer > 0:
            self.timer -= dt
            factor = self.timer / self.duration
            self.offset_x = int(np.random.uniform(-1, 1) * self.intensity * factor)
            self.offset_y = int(np.random.uniform(-1, 1) * self.intensity * factor)
        else:
            self.offset_x = 0
            self.offset_y = 0
        return self.offset_x, self.offset_y


class BootSequence:
    """启动动画序列"""

    def __init__(self, width: int, height: int):
        self.w = width
        self.h = height
        self.phase = 0  # 0=未开始, 1=正在, 2=完成
        self.timer = 0.0
        self.lines: List[str] = []
        self.current_line = 0
        self.line_timer = 0.0

        self.boot_messages = [
            "[SYS] Initializing Nav2 Stack...",
            "[SYS] Loading factory map... OK",
            "[MAP] 800x600 grid (5cm/cell) — 40m x 30m factory",
            "[MAP] 8 zones: warehouse, production, QC, loading dock",
            "[PLN] A* global planner initialized",
            "[PLN] DWA local planner ready",
            "[SEN] LaserScan: 180 rays, 8.0m range",
            "[SEN] IMU: 100Hz, quaternion mode",
            "[SEN] Odometry: differential drive model",
            "[NAV] Costmap inflation: 4 cells radius",
            "[NAV] Inspection route: 17 waypoints loaded",
            "[SYS] Particle system: 500 max particles",
            "[BOT] AGV-2000 DifferentialDrive v3.0",
            "[BOT] Max: 1.5m/s linear, 3.0rad/s angular",
            "[SYS] ══════════════════════════════════",
            "[SYS] ✅ ALL SYSTEMS NOMINAL",
            "[SYS] Factory inspection ready. Starting route.",
            "",
        ]

    def start(self):
        self.phase = 1
        self.timer = 0.0
        self.current_line = 0
        self.lines = []
        self.line_timer = 0.0

    def update(self, dt: float) -> bool:
        """更新，返回 True 表示动画完成"""
        if self.phase != 1:
            return self.phase == 2

        self.timer += dt
        self.line_timer += dt

        # 每 0.12s 一行
        if self.line_timer >= 0.12 and self.current_line < len(self.boot_messages):
            self.line_timer = 0.0
            self.lines.append(self.boot_messages[self.current_line])
            self.current_line += 1

        if self.current_line >= len(self.boot_messages):
            if self.timer > len(self.boot_messages) * 0.12 + 0.5:
                self.phase = 2
                return True

        return False

    def draw(self, surface: pygame.Surface):
        if self.phase != 1:
            return

        # 全黑背景
        surface.fill((0, 0, 0))

        try:
            font = pygame.font.SysFont("monospace", 16)
            font_lg = pygame.font.SysFont("monospace", 28, bold=True)
        except:
            font = pygame.font.Font(None, 20)
            font_lg = pygame.font.Font(None, 32)

        # 标题
        title = font_lg.render("🤖 ROS2 NAVIGATION SYSTEM", True, (0, 200, 255))
        surface.blit(title, (self.w // 2 - title.get_width() // 2, 40))

        subtitle = font.render("Autonomous Navigation Simulator v2.1", True, (100, 150, 200))
        surface.blit(subtitle, (self.w // 2 - subtitle.get_width() // 2, 80))

        # 启动日志
        y = 140
        for i, line in enumerate(self.lines):
            if "✅" in line or "NOMINAL" in line:
                color = (0, 255, 100)
            elif "[ERR]" in line:
                color = (255, 50, 50)
            elif "[SYS]" in line:
                color = (0, 200, 255)
            elif "[MAP]" in line:
                color = (255, 150, 50)
            elif "[PLN]" in line:
                color = (200, 100, 255)
            elif "[SEN]" in line:
                color = (100, 255, 150)
            elif "[NAV]" in line:
                color = (255, 100, 180)
            elif "[BOT]" in line:
                color = (60, 180, 255)
            else:
                color = (100, 100, 120)

            text = font.render(line, True, color)
            surface.blit(text, (80, y))
            y += 22

        # 光标闪烁
        if int(self.timer * 4) % 2 == 0 and self.current_line < len(self.boot_messages):
            cursor_y = 140 + len(self.lines) * 22
            pygame.draw.rect(surface, (0, 200, 255), (80, cursor_y, 10, 16))


class VignetteEffect:
    """暗角效果"""

    def __init__(self, width: int, height: int):
        self.surface = pygame.Surface((width, height), pygame.SRCALPHA)
        # 从边缘向中心渐变
        for i in range(80):
            alpha = int(80 * (1 - i / 80) ** 1.5)
            pygame.draw.rect(self.surface, (0, 0, 0, alpha),
                             (i, i, width - 2 * i, height - 2 * i))

    def apply(self, surface: pygame.Surface):
        surface.blit(self.surface, (0, 0))


class GridOverlay:
    """网格叠加层"""

    def __init__(self, width: int, height: int, spacing: int = 50):
        self.surface = pygame.Surface((width, height), pygame.SRCALPHA)
        for x in range(0, width, spacing):
            pygame.draw.line(self.surface, (0, 200, 255, 12),
                             (x, 0), (x, height))
        for y in range(0, height, spacing):
            pygame.draw.line(self.surface, (0, 200, 255, 12),
                             (0, y), (width, y))

    def apply(self, surface: pygame.Surface):
        surface.blit(self.surface, (0, 0))


class GlitchEffect:
    """偶尔的故障效果"""

    def __init__(self, width: int, height: int):
        self.w = width
        self.h = height
        self.active = False
        self.timer = 0.0
        self.next_trigger = np.random.uniform(5, 15)
        self.glitch_bars: List[Tuple[int, int, int]] = []

    def update(self, dt: float):
        self.next_trigger -= dt
        if self.next_trigger <= 0 and not self.active:
            self.active = True
            self.timer = np.random.uniform(0.05, 0.15)
            self.glitch_bars = []
            for _ in range(np.random.randint(2, 6)):
                y = np.random.randint(0, self.h)
                h = np.random.randint(2, 8)
                offset = np.random.randint(-20, 20)
                self.glitch_bars.append((y, h, offset))

        if self.active:
            self.timer -= dt
            if self.timer <= 0:
                self.active = False
                self.next_trigger = np.random.uniform(8, 20)

    def apply(self, surface: pygame.Surface):
        if not self.active:
            return
        for y, h, offset in self.glitch_bars:
            # 水平偏移条纹
            strip = surface.subsurface((0, y, self.w, min(h, self.h - y))).copy()
            surface.blit(strip, (offset, y))
