"""
ROS2 自主导航仿真系统 — 主入口 (增强版)
整合特效层: CRT扫描线 / 霓虹发光 / 雷达波 / 波形图 / 启动动画 / 屏幕震动 / 故障效果
"""
import sys
import os
import math
import time
import numpy as np
import pygame

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maps.complex_map import generate_complex_map
from core.path_planner import AStarPlanner, DWAPlanner, Pose, Velocity
from core.robot import DifferentialDriveRobot
from core.visualizer import NavigationVisualizer, Colors
from core.navigator import WaypointNavigator, NavState
from core.effects import (
    ScanlineEffect, NeonGlow, RadarSweep, WaveformChart,
    ScreenShake, BootSequence, VignetteEffect, GridOverlay, GlitchEffect
)


class CyberpunkHUD:
    """赛博朋克风格 HUD 面板"""

    def __init__(self, screen_w: int, screen_h: int):
        self.screen_w = screen_w
        self.screen_h = screen_h
        self.hud_w = 340
        self.hud_x = screen_w - self.hud_w

        # 波形图
        self.wave_vel = WaveformChart(
            self.hud_x + 15, screen_h - 220, 310, 50,
            "LINEAR VEL (m/s)", color=(0, 255, 180)
        )
        self.wave_vel.max_val = 1.5
        self.wave_vel.min_val = -0.3

        self.wave_ang = WaveformChart(
            self.hud_x + 15, screen_h - 160, 310, 50,
            "ANGULAR VEL (rad/s)", color=(255, 100, 180)
        )
        self.wave_ang.max_val = 3.0
        self.wave_ang.min_val = -3.0

    def update(self, v: float, w: float):
        self.wave_vel.add_point(v)
        self.wave_ang.add_point(w)

    def draw(self, surface: pygame.Surface, telemetry: dict,
             nav_status: str, waypoints: list, current_wp: int,
             time_val: float, log_messages: list):
        # 半透明背景
        hud_bg = pygame.Surface((self.hud_w, self.screen_h), pygame.SRCALPHA)
        hud_bg.fill((5, 5, 15, 230))
        surface.blit(hud_bg, (self.hud_x, 0))

        # 左边框霓虹线
        for i in range(0, self.screen_h, 4):
            alpha = int(80 + 40 * math.sin(time_val * 3 + i * 0.02))
            try:
                pygame.draw.line(surface, (0, 200, 255, alpha),
                                 (self.hud_x, i), (self.hud_x, min(i + 2, self.screen_h)), 1)
            except:
                pass

        y = 15
        xt = self.hud_x + 15

        try:
            font_title = pygame.font.SysFont("monospace", 20, bold=True)
            font_lg = pygame.font.SysFont("monospace", 16, bold=True)
            font_md = pygame.font.SysFont("monospace", 13)
            font_sm = pygame.font.SysFont("monospace", 11)
        except:
            font_title = pygame.font.Font(None, 24)
            font_lg = pygame.font.Font(None, 20)
            font_md = pygame.font.Font(None, 17)
            font_sm = pygame.font.Font(None, 15)

        # 标题 — 闪烁效果
        pulse = (math.sin(time_val * 2) + 1) / 2
        title_color = (int(0 + 60 * pulse), int(180 + 75 * pulse), 255)
        title = font_title.render("≡ NAV SYSTEM", True, title_color)
        surface.blit(title, (xt, y))
        y += 32

        # 状态 — 脉冲指示灯
        status_colors = {
            'NAVIGATING': (0, 255, 100),
            'PLANNING': (255, 200, 50),
            'ARRIVED': (0, 200, 255),
            'IDLE': (100, 100, 120),
            'STUCK': (255, 50, 50),
            'REPLAN': (255, 150, 50),
        }
        sc = status_colors.get(nav_status, (150, 150, 150))

        # 脉冲圆点
        dot_r = int(4 + 2 * math.sin(time_val * 6))
        pygame.draw.circle(surface, sc, (xt + dot_r + 2, y + 8), dot_r)
        status_text = font_lg.render(f" {nav_status}", True, sc)
        surface.blit(status_text, (xt + dot_r * 2 + 8, y))
        y += 30

        # 分割线 — 霓虹
        NeonGlow.draw_line_glow(surface, (xt, y), (self.hud_x + self.hud_w - 15, y),
                                 (0, 200, 255), width=1, glow_radius=3, intensity=0.3)
        y += 12

        # 数据面板
        sections = [
            ("◆ POSE", [
                ("X", f"{telemetry.get('x', 0):.3f} m"),
                ("Y", f"{telemetry.get('y', 0):.3f} m"),
                ("θ", f"{math.degrees(telemetry.get('theta', 0)):.1f}°"),
                ("", ""),
                ("V", f"{telemetry.get('v', 0):+.3f} m/s"),
                ("W", f"{telemetry.get('w', 0):+.3f} rad/s"),
            ]),
            ("◆ NAVIGATION", [
                ("Algorithm", "A* + DWA"),
                ("Plan", f"{telemetry.get('plan_time', 0)*1000:.1f} ms"),
                ("Distance", f"{telemetry.get('total_dist', 0):.2f} m"),
                ("Waypoints", f"{current_wp}/{len(waypoints)}"),
                ("Replans", str(telemetry.get('replans', 0))),
                ("Collisions", str(telemetry.get('collisions', 0))),
            ]),
            ("◆ SENSORS", [
                ("Laser", f"{len(telemetry.get('scan_ranges', []))} pts"),
                ("Rate", f"{telemetry.get('scan_hz', 0):.0f} Hz"),
            ]),
        ]

        for sec_title, items in sections:
            st = font_md.render(sec_title, True, (0, 200, 255))
            surface.blit(st, (xt, y))
            y += 20

            for label, value in items:
                if not label and not value:
                    y += 5
                    continue
                lt = font_sm.render(f"  {label}:", True, (120, 140, 160))
                vt = font_sm.render(value, True, (200, 220, 240))
                surface.blit(lt, (xt, y))
                surface.blit(vt, (xt + 100, y))
                y += 16

            y += 8

        # 速度条 — 动画
        y += 5
        bar_label = font_md.render("◆ THRUST", True, (0, 200, 255))
        surface.blit(bar_label, (xt, y))
        y += 20

        v = abs(telemetry.get('v', 0))
        w = abs(telemetry.get('w', 0))

        # 速度条 (带辉光)
        self._draw_cyber_bar(surface, xt, y, 180, 8, v / 1.5,
                              (0, 255, 150), time_val, "V")
        y += 16
        self._draw_cyber_bar(surface, xt, y, 180, 8, w / 3.0,
                              (255, 100, 180), time_val, "W")
        y += 20

        # 波形图
        self.wave_vel.draw(surface, time_val)
        self.wave_ang.draw(surface, time_val)

        # 日志
        y_log = self.screen_h - 280
        log_label = font_md.render("◆ LOG", True, (0, 200, 255))
        surface.blit(log_label, (xt, y_log))
        y_log += 18

        for i, msg in enumerate(reversed(log_messages)):
            alpha_factor = 1 - i / max(len(log_messages), 1)
            color = (int(100 + 80 * alpha_factor), int(100 + 80 * alpha_factor),
                     int(120 + 80 * alpha_factor))
            lt = font_sm.render(msg, True, color)
            surface.blit(lt, (xt, y_log))
            y_log += 14

    def _draw_cyber_bar(self, surface, x, y, width, height, ratio,
                         color, time_val, label):
        ratio = max(0, min(1, ratio))

        # 背景
        pygame.draw.rect(surface, (15, 15, 25), (x, y, width, height))
        # 前景
        bar_w = int(width * ratio)
        if bar_w > 0:
            pygame.draw.rect(surface, color, (x, y, bar_w, height))
            # 辉光
            glow = pygame.Surface((bar_w, height * 3), pygame.SRCALPHA)
            for i in range(height * 3):
                alpha = int(20 * (1 - abs(i - height * 1.5) / (height * 1.5)))
                pygame.draw.line(glow, (*color, max(0, alpha)),
                                 (0, i), (bar_w, i))
            surface.blit(glow, (x, y - height))

        # 边框脉冲
        border_alpha = int(60 + 40 * math.sin(time_val * 4))
        pygame.draw.rect(surface, (*color, border_alpha),
                         (x, y, width, height), 1)

        # 标签
        try:
            font = pygame.font.SysFont("monospace", 10)
        except:
            font = pygame.font.Font(None, 14)
        lt = font.render(f"{label} {ratio*100:.0f}%", True, (180, 200, 220))
        surface.blit(lt, (x + width + 8, y - 2))


class SimulationApp:
    """仿真主应用 (增强版)"""

    def __init__(self):
        pygame.init()
        self.clock = pygame.time.Clock()
        self.running = True
        self.paused = False
        self.boot_done = False

        WIDTH, HEIGHT = 1600, 900

        # 生成地图
        print("🏗️  Generating complex map...")
        self.map_data = generate_complex_map(seed=42)
        print(f"   Map: {self.map_data['width']}x{self.map_data['height']}")
        print(f"   Rooms: {len(self.map_data['rooms'])}, Doors: {len(self.map_data['doors'])}")

        # 可视化
        self.viz = NavigationVisualizer(WIDTH, HEIGHT)
        self.viz.load_map(self.map_data)
        self.viz.show_costmap = False
        self.viz.show_laser = True

        # 规划器
        costmap = self.map_data['costmap']
        resolution = self.map_data['resolution']
        self.obstacle_mask = np.isin(self.map_data['grid'], [1, 2, 3, 5])

        self.a_star = AStarPlanner(costmap, resolution)
        self.dwa = DWAPlanner(costmap, resolution)

        # 机器人
        spawn = self.map_data['spawn_points'][0] if self.map_data['spawn_points'] else (100, 100)
        self.robot = DifferentialDriveRobot(
            spawn[0] * resolution, spawn[1] * resolution, theta=math.pi / 4
        )

        # 导航器
        self.navigator = WaypointNavigator(
            self.a_star, self.dwa, self.robot, resolution
        )

        # 航点预设
        self.waypoint_presets = self._generate_waypoint_presets()

        # ─── 特效层 ───
        self.scanlines = ScanlineEffect(WIDTH, HEIGHT)
        self.vignette = VignetteEffect(WIDTH, HEIGHT)
        self.grid_overlay = GridOverlay(WIDTH, HEIGHT, spacing=60)
        self.screen_shake = ScreenShake()
        self.boot = BootSequence(WIDTH, HEIGHT)
        self.glitch = GlitchEffect(WIDTH, HEIGHT)

        # 赛博 HUD
        self.cyber_hud = CyberpunkHUD(WIDTH, HEIGHT)

        # 雷达 (左上角小雷达)
        self.radar = RadarSweep(90, 85, 55)

        # 传感器
        self.scan_data = []
        self.scan_timer = 0.0

        # 启动动画
        self.boot.start()
        self.boot_start_time = time.time()

        # 预先加载航点
        self._pending_preset = 0

    def _generate_waypoint_presets(self) -> dict:
        resolution = self.map_data['resolution']
        presets = {}

        presets[0] = [
            (50, 30), (30, 67), (30, 100), (65, 100),
            (65, 67), (100, 67), (100, 100), (150, 100),
            (150, 160), (100, 160), (50, 160),
        ]
        presets[1] = [
            (30, 55), (30, 67), (68, 67), (68, 130),
            (120, 130), (120, 67), (150, 67), (150, 160),
        ]
        presets[2] = [
            (15, 80), (15, 115), (60, 115), (60, 80), (35, 95),
        ]
        presets[3] = [
            (15, 155), (180, 155), (15, 170), (180, 170), (100, 160),
        ]
        presets[4] = [
            (85, 75), (130, 75), (130, 100), (85, 100),
            (85, 120), (130, 120),
        ]
        for i in range(5, 9):
            num = np.random.randint(4, 8)
            pts = [(np.random.randint(15, 185), np.random.randint(15, 185))
                   for _ in range(num)]
            presets[i] = pts

        # 转换为世界坐标
        for k in presets:
            presets[k] = [(x * resolution, y * resolution) for x, y in presets[k]]

        return presets

    def _start_navigation(self, preset_idx: int):
        if preset_idx not in self.waypoint_presets:
            return

        waypoints = self.waypoint_presets[preset_idx]
        spawn = self.map_data['spawn_points'][0] if self.map_data['spawn_points'] else (100, 100)
        resolution = self.map_data['resolution']

        self.robot = DifferentialDriveRobot(
            spawn[0] * resolution, spawn[1] * resolution,
            theta=math.atan2(
                waypoints[0][1] - spawn[1] * resolution,
                waypoints[0][0] - spawn[0] * resolution
            )
        )
        self.navigator.robot = self.robot
        self.navigator.set_waypoints(waypoints)
        self.navigator.start()

        self.viz.waypoints = waypoints
        self.viz.current_waypoint_idx = 0
        self.viz.planning_algorithm = "A* + DWA"
        self.viz.add_log(f"Preset {preset_idx + 1} started ({len(waypoints)} WP)")

    def run(self):
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            dt = min(dt, 0.05)

            self.viz.time += dt
            self.viz.frame_count += 1
            self.viz.fps_history.append(self.clock.get_fps())

            # ─── 启动动画 ───
            if not self.boot_done:
                done = self.boot.update(dt)
                if done:
                    self.boot_done = True
                    self._start_navigation(self._pending_preset)
                    self.viz.add_log("System online")

                self._handle_events()
                self.boot.draw(self.viz.screen)
                pygame.display.flip()
                continue

            self._handle_events()

            if not self.paused:
                self._update_navigation(dt)
                self._update_sensors(dt)
                self.viz.particles.update(dt)
                self.glitch.update(dt)

            # 屏幕震动
            shake_x, shake_y = self.screen_shake.update(dt)

            # 渲染
            self._render(shake_x, shake_y)

        pygame.quit()

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self._start_navigation(0)
                elif event.key == pygame.K_c:
                    self.viz.show_costmap = not self.viz.show_costmap
                elif event.key == pygame.K_l:
                    self.viz.show_laser = not self.viz.show_laser
                elif event.key == pygame.K_t:
                    self.viz.show_trajectory = not self.viz.show_trajectory
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                    self.viz.target_scale *= 1.2
                elif event.key == pygame.K_MINUS:
                    self.viz.target_scale /= 1.2
                elif pygame.K_1 <= event.key <= pygame.K_9:
                    self._start_navigation(event.key - pygame.K_1)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    self.viz.is_dragging = True
                    self.viz.drag_start = event.pos
                elif event.button == 4:
                    self.viz.target_scale *= 1.1
                elif event.button == 5:
                    self.viz.target_scale /= 1.1

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.viz.is_dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if self.viz.is_dragging:
                    dx = event.pos[0] - self.viz.drag_start[0]
                    dy = event.pos[1] - self.viz.drag_start[1]
                    self.viz.view_offset_x += dx
                    self.viz.view_offset_y += dy
                    self.viz.drag_start = event.pos

        scale_diff = self.viz.target_scale - self.viz.view_scale
        self.viz.view_scale += scale_diff * 0.15

    def _update_navigation(self, dt: float):
        nav = self.navigator
        resolution = self.map_data['resolution']

        if nav.state == NavState.IDLE:
            return

        elif nav.state == NavState.PLANNING:
            nav._plan_to_current_waypoint()
            if nav.state == NavState.NAVIGATING:
                self.viz.add_log(f"Path → WP {nav.current_waypoint_idx + 1}")
                self.viz.global_path = nav.global_path

        elif nav.state == NavState.NAVIGATING:
            current_pos = (self.robot.state.x, self.robot.state.y)
            target = nav._get_current_target(current_pos)

            if target is None:
                nav.current_waypoint_idx += 1
                self.viz.current_waypoint_idx = nav.current_waypoint_idx
                self.viz.add_log(f"✅ WP {nav.current_waypoint_idx} reached")
                self.screen_shake.trigger(3.0, 0.1)

                # 粒子烟花
                self.viz.particles.emit(
                    self.robot.state.x, self.robot.state.y,
                    count=40, color=(100, 255, 150),
                    spread=4.0, speed=50.0, lifetime=1.2
                )

                if nav.current_waypoint_idx >= len(nav.waypoints):
                    nav.state = NavState.ARRIVED
                    self.viz.nav_status = "ARRIVED"
                    self.viz.add_log("🎉 ALL WAYPOINTS REACHED!")
                    self.screen_shake.trigger(8.0, 0.3)
                    self.viz.particles.emit(
                        self.robot.state.x, self.robot.state.y,
                        count=100, color=(255, 200, 50),
                        spread=6.0, speed=80.0, lifetime=2.0
                    )
                else:
                    nav.state = NavState.PLANNING
                return

            # DWA
            goal_pose = Pose(target[0], target[1], 0.0)
            current_pose = Pose(self.robot.state.x, self.robot.state.y, self.robot.state.theta)
            current_vel = Velocity(self.robot.state.v, self.robot.state.w)

            vel_cmd, local_traj = dwa = self.dwa.plan(current_pose, goal_pose, current_vel)
            self.robot.update(vel_cmd.v, vel_cmd.w, dt)

            if nav.last_positions:
                last = nav.last_positions[-1]
                dist = math.sqrt((current_pos[0] - last[0]) ** 2 +
                                 (current_pos[1] - last[1]) ** 2)
                nav.total_distance += dist
            nav.last_positions.append(current_pos)
            if len(nav.last_positions) > 100:
                nav.last_positions.pop(0)

            if local_traj:
                self.viz.local_path = [(p.x, p.y) for p in local_traj]

            if nav._check_stuck():
                nav.state = NavState.REPLAN
                nav.replan_count += 1
                self.screen_shake.trigger(5.0, 0.2)
                self.viz.add_log("⚠️ STUCK — Replanning...")
                self.viz.particles.emit(
                    self.robot.state.x, self.robot.state.y,
                    count=20, color=(255, 100, 50),
                    spread=3.0, speed=30.0, lifetime=0.8
                )

        elif nav.state == NavState.REPLAN:
            nav._plan_to_current_waypoint()
            if nav.state == NavState.NAVIGATING:
                self.viz.add_log(f"Replanned (#{nav.replan_count})")
                self.viz.global_path = nav.global_path

        # 更新 HUD 波形
        self.cyber_hud.update(self.robot.state.v, self.robot.state.w)

        # 更新遥测
        self.viz.telemetry = {
            'x': self.robot.state.x, 'y': self.robot.state.y,
            'theta': self.robot.state.theta,
            'v': self.robot.state.v, 'w': self.robot.state.w,
            'plan_time': 0.001, 'total_dist': nav.total_distance,
            'waypoints_done': nav.current_waypoint_idx,
            'collisions': nav.collision_count,
            'replans': nav.replan_count,
            'scan_ranges': self.scan_data, 'scan_hz': 10,
        }
        self.viz.nav_status = nav.state.value

    def _update_sensors(self, dt: float):
        self.scan_timer += dt
        if self.scan_timer >= 0.1:
            self.scan_timer = 0.0
            scan = self.robot.get_laser_scan(
                self.obstacle_mask, self.map_data['resolution'],
                num_rays=180, max_range=8.0
            )
            self.scan_data = scan.ranges

    def _render(self, shake_x: int = 0, shake_y: int = 0):
        screen = self.viz.screen
        screen.fill(Colors.BG)

        # 应用震动偏移 — 需要创建一个临时 surface
        if shake_x != 0 or shake_y != 0:
            # 先渲染到临时 surface 再偏移
            pass  # pygame 不支持简单偏移，直接在坐标上加偏移

        # 地图
        self.viz.draw_map()

        # 代价地图
        self.viz.draw_costmap()

        # 网格叠加
        self.grid_overlay.apply(screen)

        # 全局路径
        if self.viz.global_path:
            self.viz.draw_global_path(self.viz.global_path)

        # 局部路径
        if self.viz.local_path:
            self.viz.draw_local_path(self.viz.local_path)

        # 航点
        self.viz.draw_waypoints(self.viz.waypoints, self.viz.current_waypoint_idx)

        # 轨迹
        if self.robot.trajectory:
            self.viz.draw_trajectory(self.robot.trajectory)

        # 激光雷达
        self.viz.draw_laser_scan(
            self.scan_data, self.robot.state.x,
            self.robot.state.y, self.robot.state.theta
        )

        # 机器人
        self.viz.draw_robot(self.robot.state.x, self.robot.state.y,
                            self.robot.state.theta)

        # 粒子
        self.viz.particles.draw(screen, self.viz.view_offset_x,
                                 self.viz.view_offset_y, self.viz.view_scale)

        # ─── 特效层 ───
        self.vignette.apply(screen)
        self.scanlines.apply(screen)
        self.glitch.apply(screen)

        # 雷达
        self.radar.update(1.0 / 60)
        self.radar.draw(screen, (0, 255, 100))

        # 赛博 HUD
        self.cyber_hud.draw(
            screen, self.viz.telemetry, self.viz.nav_status,
            self.viz.waypoints, self.viz.current_waypoint_idx,
            self.viz.time, self.viz.log_messages
        )

        # 小地图
        self.viz.draw_minimap(self.robot.state.x, self.robot.state.y,
                               self.robot.state.theta)

        # 标题栏
        self.viz.draw_title_bar()

        # 底部控制提示
        self.viz.draw_controls_help()

        # 暂停
        if self.paused:
            pause_surf = pygame.Surface((350, 70), pygame.SRCALPHA)
            pause_surf.fill((0, 0, 0, 200))
            screen.blit(pause_surf,
                        (self.viz.screen_width // 2 - 175,
                         self.viz.screen_height // 2 - 35))
            try:
                font = pygame.font.SysFont("monospace", 28, bold=True)
            except:
                font = pygame.font.Font(None, 32)
            pt = font.render("⏸  PAUSED", True, Colors.HUD_WARN)
            screen.blit(pt, (self.viz.screen_width // 2 - 90,
                             self.viz.screen_height // 2 - 18))

        pygame.display.flip()


def main():
    print("=" * 60)
    print("  🤖 ROS2 Autonomous Navigation Simulator — CYBERPUNK")
    print("  Nav2 + SLAM + DWA + Particle FX + CRT Shader")
    print("=" * 60)

    app = SimulationApp()
    app.run()


if __name__ == "__main__":
    main()
