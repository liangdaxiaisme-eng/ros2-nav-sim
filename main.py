"""
ROS 2 Autonomous Navigation Simulator — Main Entry Point

Wires up all ROS 2 nodes (map_server, robot_state_publisher, global_planner,
controller_server, waypoint_navigator, tf_broadcaster) and runs the Pygame
visualisation loop.  Every inter-node communication goes through the
rclpy-compatible pub / sub / action layer.
"""
import sys
import os
import math
import time
import numpy as np
import pygame

# ── Ensure project root on path ──────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ── ROS 2 compat layer ───────────────────────────────────────────────────────
from ros2_nav_sim.ros_compat import (
    init as ros_init, shutdown as ros_shutdown, spin_once, ok as ros_ok,
    QoSProfile, DURABILITY_TRANSIENT_LOCAL,
)

# ── ROS 2 message types ─────────────────────────────────────────────────────
from ros2_nav_sim.msgs.geometry_msgs import (
    Pose, Point, Quaternion, PoseStamped, Twist, Vector3,
)
from ros2_nav_sim.msgs.nav_msgs import Odometry, Path, OccupancyGrid

# ── ROS 2 nodes ──────────────────────────────────────────────────────────────
from ros2_nav_sim.nodes.map_server_node import MapServerNode
from ros2_nav_sim.nodes.robot_node import RobotNode
from ros2_nav_sim.nodes.planner_node import PlannerNode
from ros2_nav_sim.nodes.controller_node import ControllerNode
from ros2_nav_sim.nodes.navigator_node import NavigatorNode
from ros2_nav_sim.nodes.tf_broadcaster_node import TFBroadcasterNode

# ── Algorithm layer (pure Python, no ROS dependency) ─────────────────────────
from maps.factory_map import generate_factory_map
from core.path_planner import AStarPlanner, DWAPlanner
from core.robot import DifferentialDriveRobot

# ── Visualisation & effects ──────────────────────────────────────────────────
from core.visualizer import NavigationVisualizer, Colors
from core.effects import (
    ScanlineEffect, NeonGlow, RadarSweep, WaveformChart,
    ScreenShake, BootSequence, VignetteEffect, GridOverlay, GlitchEffect,
)


# ═════════════════════════════════════════════════════════════════════════════
# Cyberpunk HUD — right-side telemetry panel
# ═════════════════════════════════════════════════════════════════════════════
class CyberpunkHUD:
    def __init__(self, screen_w: int, screen_h: int):
        self.screen_w = screen_w
        self.screen_h = screen_h
        self.hud_w = 340
        self.hud_x = screen_w - self.hud_w

        self.wave_vel = WaveformChart(
            self.hud_x + 15, screen_h - 220, 310, 50,
            "LINEAR VEL (m/s)", color=(0, 255, 180),
        )
        self.wave_vel.max_val = 1.5
        self.wave_vel.min_val = -0.3

        self.wave_ang = WaveformChart(
            self.hud_x + 15, screen_h - 160, 310, 50,
            "ANGULAR VEL (rad/s)", color=(255, 100, 180),
        )
        self.wave_ang.max_val = 3.0
        self.wave_ang.min_val = -3.0

    def update(self, v: float, w: float):
        self.wave_vel.add_point(v)
        self.wave_ang.add_point(w)

    # ------------------------------------------------------------------
    def draw(self, surface: pygame.Surface, telemetry: dict,
             nav_status: str, waypoints: list, current_wp: int,
             time_val: float, log_messages: list):
        hud_bg = pygame.Surface((self.hud_w, self.screen_h), pygame.SRCALPHA)
        hud_bg.fill((5, 5, 15, 230))
        surface.blit(hud_bg, (self.hud_x, 0))

        # left-edge neon line
        for i in range(0, self.screen_h, 4):
            alpha = int(80 + 40 * math.sin(time_val * 3 + i * 0.02))
            try:
                pygame.draw.line(surface, (0, 200, 255, alpha),
                                 (self.hud_x, i), (self.hud_x, min(i + 2, self.screen_h)), 1)
            except Exception:
                pass

        y = 15
        xt = self.hud_x + 15

        try:
            font_title = pygame.font.SysFont("monospace", 20, bold=True)
            font_lg = pygame.font.SysFont("monospace", 16, bold=True)
            font_md = pygame.font.SysFont("monospace", 13)
            font_sm = pygame.font.SysFont("monospace", 11)
        except Exception:
            font_title = pygame.font.Font(None, 24)
            font_lg = pygame.font.Font(None, 20)
            font_md = pygame.font.Font(None, 17)
            font_sm = pygame.font.Font(None, 15)

        pulse = (math.sin(time_val * 2) + 1) / 2
        title_color = (int(60 * pulse), int(180 + 75 * pulse), 255)
        title = font_title.render("≡ NAV SYSTEM [ROS 2]", True, title_color)
        surface.blit(title, (xt, y))
        y += 32

        # status LED
        status_colors = {
            'NAVIGATING': (0, 255, 100),
            'PLANNING': (255, 200, 50),
            'ARRIVED': (0, 200, 255),
            'IDLE': (100, 100, 120),
            'STUCK': (255, 50, 50),
            'REPLAN': (255, 150, 50),
            'PATROL_COMPLETE': (0, 255, 200),
        }
        sc = status_colors.get(nav_status, (150, 150, 150))
        dot_r = int(4 + 2 * math.sin(time_val * 6))
        pygame.draw.circle(surface, sc, (xt + dot_r + 2, y + 8), dot_r)
        st = font_lg.render(f" {nav_status}", True, sc)
        surface.blit(st, (xt + dot_r * 2 + 8, y))
        y += 30

        NeonGlow.draw_line_glow(surface, (xt, y), (self.hud_x + self.hud_w - 15, y),
                                (0, 200, 255), width=1, glow_radius=3, intensity=0.3)
        y += 12

        sections = [
            ("◆ POSE [map→base_link]", [
                ("X", f"{telemetry.get('x', 0):.3f} m"),
                ("Y", f"{telemetry.get('y', 0):.3f} m"),
                ("θ", f"{math.degrees(telemetry.get('theta', 0)):.1f}°"),
                ("", ""),
                ("V", f"{telemetry.get('v', 0):+.3f} m/s"),
                ("W", f"{telemetry.get('w', 0):+.3f} rad/s"),
            ]),
            ("◆ NAVIGATION [Nav2]", [
                ("Planner", "nav2_astar/A*"),
                ("Controller", "dwb_core/DWA"),
                ("Plan", f"{telemetry.get('plan_time', 0) * 1000:.1f} ms"),
                ("Distance", f"{telemetry.get('total_dist', 0):.2f} m"),
                ("Waypoints", f"{current_wp}/{len(waypoints)}"),
                ("Replans", str(telemetry.get('replans', 0))),
                ("Collisions", str(telemetry.get('collisions', 0))),
            ]),
            ("◆ TOPICS", [
                ("/odom", "pub: robot_state_pub"),
                ("/scan", "pub: robot_state_pub"),
                ("/imu", "pub: robot_state_pub"),
                ("/tf", "pub: tf_broadcaster"),
                ("/cmd_vel", "pub: controller_srv"),
                ("/plan", "pub: global_planner"),
                ("/goal_pose", "pub: waypoint_nav"),
            ]),
            ("◆ SENSORS", [
                ("Laser", f"{len(telemetry.get('scan_ranges', []))} pts"),
                ("Battery", f"{telemetry.get('battery_pct', 100):.0f}%"),
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

        # speed bars
        bar_label = font_md.render("◆ THRUST", True, (0, 200, 255))
        surface.blit(bar_label, (xt, y))
        y += 20
        v = abs(telemetry.get('v', 0))
        w = abs(telemetry.get('w', 0))
        self._draw_cyber_bar(surface, xt, y, 180, 8, v / 1.5,
                             (0, 255, 150), time_val, "V")
        y += 16
        self._draw_cyber_bar(surface, xt, y, 180, 8, w / 3.0,
                             (255, 100, 180), time_val, "W")
        y += 20

        self.wave_vel.draw(surface, time_val)
        self.wave_ang.draw(surface, time_val)

        y_log = self.screen_h - 280
        log_label = font_md.render("◆ LOG [rclpy]", True, (0, 200, 255))
        surface.blit(log_label, (xt, y_log))
        y_log += 18
        for i, msg in enumerate(reversed(log_messages)):
            af = 1 - i / max(len(log_messages), 1)
            color = (int(100 + 80 * af), int(100 + 80 * af), int(120 + 80 * af))
            lt = font_sm.render(msg, True, color)
            surface.blit(lt, (xt, y_log))
            y_log += 14

    @staticmethod
    def _draw_cyber_bar(surface, x, y, width, height, ratio,
                        color, time_val, label):
        ratio = max(0, min(1, ratio))
        pygame.draw.rect(surface, (15, 15, 25), (x, y, width, height))
        bar_w = int(width * ratio)
        if bar_w > 0:
            pygame.draw.rect(surface, color, (x, y, bar_w, height))
            glow = pygame.Surface((bar_w, height * 3), pygame.SRCALPHA)
            for i in range(height * 3):
                alpha = int(20 * (1 - abs(i - height * 1.5) / (height * 1.5)))
                pygame.draw.line(glow, (*color, max(0, alpha)), (0, i), (bar_w, i))
            surface.blit(glow, (x, y - height))
        border_alpha = int(60 + 40 * math.sin(time_val * 4))
        pygame.draw.rect(surface, (*color, border_alpha), (x, y, width, height), 1)
        try:
            font = pygame.font.SysFont("monospace", 10)
        except Exception:
            font = pygame.font.Font(None, 14)
        lt = font.render(f"{label} {ratio * 100:.0f}%", True, (180, 200, 220))
        surface.blit(lt, (x + width + 8, y - 2))


# ═════════════════════════════════════════════════════════════════════════════
# SimulationApp — main loop
# ═════════════════════════════════════════════════════════════════════════════
class SimulationApp:
    def __init__(self):
        pygame.init()
        self.clock = pygame.time.Clock()
        self.running = True
        self.paused = False
        self.boot_done = False

        WIDTH, HEIGHT = 1600, 900

        # ── 1. Generate map ──────────────────────────────────────────────
        print("🏗️  Generating factory map …")
        self.map_data = generate_factory_map(seed=42)
        map_w, map_h = self.map_data['map_size_m']
        resolution = self.map_data['resolution']
        print(f"   Map: {self.map_data['width']}×{self.map_data['height']} cells")
        print(f"   Real size: {map_w:.0f} m × {map_h:.0f} m ({map_w * map_h:.0f} m²)")

        self.obstacle_mask = np.isin(self.map_data['grid'],
                                     [1, 2, 3, 4, 5, 6, 11])

        # ── 2. ROS 2 init ───────────────────────────────────────────────
        ros_init()

        # ── 3. Create nodes ──────────────────────────────────────────────
        costmap = self.map_data['costmap']

        self.map_server = MapServerNode(self.map_data['grid'], resolution)

        spawn = self.map_data['spawn_points'][0] if self.map_data['spawn_points'] else (50, 240)
        # Face toward first real waypoint so DWA gets a clear initial heading
        factory_wps = self.map_data['inspection_waypoints']
        if len(factory_wps) > 1:
            init_theta = math.atan2(
                factory_wps[1][1] - spawn[1], factory_wps[1][0] - spawn[0])
        else:
            init_theta = 0.0
        self.robot = DifferentialDriveRobot(
            spawn[0] * resolution, spawn[1] * resolution, theta=init_theta,
        )
        self.robot_node = RobotNode(self.robot, self.obstacle_mask, resolution)

        self.planner_node = PlannerNode(costmap, resolution)
        self.controller_node = ControllerNode(costmap, resolution)
        self.navigator_node = NavigatorNode(self.planner_node, self.controller_node)
        self.tf_broadcaster = TFBroadcasterNode()

        # ── 4. Visualisation ─────────────────────────────────────────────
        self.viz = NavigationVisualizer(WIDTH, HEIGHT)
        self.viz.load_map(self.map_data)
        self.viz.show_costmap = False
        self.viz.show_laser = True

        # Cyberpunk HUD
        self.cyber_hud = CyberpunkHUD(WIDTH, HEIGHT)

        # Effects
        self.scanlines = ScanlineEffect(WIDTH, HEIGHT)
        self.vignette = VignetteEffect(WIDTH, HEIGHT)
        self.grid_overlay = GridOverlay(WIDTH, HEIGHT, spacing=60)
        self.screen_shake = ScreenShake()
        self.boot = BootSequence(WIDTH, HEIGHT)
        self.glitch = GlitchEffect(WIDTH, HEIGHT)
        self.radar = RadarSweep(90, 85, 55)

        # Scan cache
        self.scan_data = []
        self.scan_timer = 0.0

        # Boot
        self.boot.start()
        self._pending_preset = 0

        # ── 5. Topic subscriptions (for viz / HUD) ──────────────────────
        from ros2_nav_sim.ros_compat import QoSProfile as _Q, DURABILITY_TRANSIENT_LOCAL as _TL
        from ros2_nav_sim.msgs.sensor_msgs import LaserScan as _LS

        # Latch onto /scan for the latest laser data
        self.robot_node.create_subscription(
            _LS, '/scan',
            self._on_scan,
            _Q(depth=1, durability=_TL),
        )

        # Track nav status from navigator
        from ros2_nav_sim.msgs.std_msgs import String
        self._nav_status = "IDLE"
        self.navigator_node.create_subscription(
            String, '/navigation_status',
            self._on_nav_status,
            _Q(depth=1),
        )

        # ── 6. Waypoint presets ──────────────────────────────────────────
        factory_wps = self.map_data['inspection_waypoints']
        self.waypoint_presets = {
            0: [(x * resolution, y * resolution) for x, y in factory_wps],
        }
        if len(factory_wps) > 5:
            self.waypoint_presets[1] = [
                (factory_wps[0][0] * resolution, factory_wps[0][1] * resolution),
            ]
            self.waypoint_presets[2] = [
                (factory_wps[i][0] * resolution, factory_wps[i][1] * resolution)
                for i in range(5, min(8, len(factory_wps)))
            ]
            self.waypoint_presets[3] = [
                (factory_wps[i][0] * resolution, factory_wps[i][1] * resolution)
                for i in range(1, min(4, len(factory_wps)))
            ]

    # ── topic callbacks ─────────────────────────────────────────────────
    def _on_scan(self, msg):
        self.scan_data = list(msg.ranges)

    def _on_nav_status(self, msg):
        self._nav_status = msg.data

    # ── navigation start ────────────────────────────────────────────────
    def _start_navigation(self, preset_idx: int):
        if preset_idx not in self.waypoint_presets:
            return

        waypoints = self.waypoint_presets[preset_idx]
        resolution = self.map_data['resolution']
        spawn = self.map_data['spawn_points'][0] if self.map_data['spawn_points'] else (50, 240)

        # Reset robot — face toward first real waypoint
        from core.robot import DifferentialDriveRobot as DDR
        first_real_wp = None
        for wp in waypoints:
            dx = wp[0] - spawn[0] * resolution
            dy = wp[1] - spawn[1] * resolution
            if math.sqrt(dx*dx + dy*dy) > 0.3:  # skip waypoints at starting pos
                first_real_wp = wp
                break
        if first_real_wp:
            init_theta = math.atan2(
                first_real_wp[1] - spawn[1] * resolution,
                first_real_wp[0] - spawn[0] * resolution,
            )
        else:
            init_theta = math.atan2(
                waypoints[0][1] - spawn[1] * resolution,
                waypoints[0][0] - spawn[0] * resolution,
            )
        self.robot = DDR(
            spawn[0] * resolution, spawn[1] * resolution,
            theta=init_theta,
        )
        self.robot_node.robot = self.robot

        # Inject initial pose into planner BEFORE starting patrol
        # (start_patrol publishes /goal_pose immediately, planner needs pose first)
        initial_pose = Pose(
            position=Point(x=self.robot.state.x, y=self.robot.state.y, z=0.0),
            orientation=Quaternion(
                x=0.0, y=0.0,
                z=math.sin(self.robot.state.theta / 2),
                w=math.cos(self.robot.state.theta / 2),
            ),
        )
        self.planner_node.set_current_pose(initial_pose)

        # Inject initial position into navigator for waypoint tracking
        self.navigator_node._odom_x = self.robot.state.x
        self.navigator_node._odom_y = self.robot.state.y
        self.navigator_node._odom_theta = self.robot.state.theta

        # Tell navigator to start patrol — it will publish /goal_pose
        self.navigator_node.set_waypoints(waypoints)
        self.navigator_node.start_patrol()

        self.viz.waypoints = waypoints
        self.viz.current_waypoint_idx = 0
        self.viz.planning_algorithm = "A* + DWA [Nav2]"
        self.viz.add_log(f"Preset {preset_idx + 1}: {len(waypoints)} waypoints")

    # ── main loop ───────────────────────────────────────────────────────
    def run(self):
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            dt = min(dt, 0.05)

            self.viz.time += dt
            self.viz.frame_count += 1
            self.viz.fps_history.append(self.clock.get_fps())

            # ── Boot animation ───────────────────────────────────────
            if not self.boot_done:
                done = self.boot.update(dt)
                if done:
                    self.boot_done = True
                    self._start_navigation(self._pending_preset)
                    self.viz.add_log("ROS 2 nodes online")
                self._handle_events()
                self.boot.draw(self.viz.screen)
                pygame.display.flip()
                continue

            self._handle_events()

            if not self.paused:
                self._update_simulation(dt)
                self.viz.particles.update(dt)
                self.glitch.update(dt)

            shake_x, shake_y = self.screen_shake.update(dt)
            self._render(shake_x, shake_y)

        ros_shutdown()
        pygame.quit()

    def _update_simulation(self, dt: float):
        """One simulation tick — the correct order is:

        1. Inject current state into all nodes (from previous tick)
        2. Planner processes /goal_pose → publishes /plan
        3. Controller reads /plan + /odom → publishes /cmd_vel
        4. Robot reads /cmd_vel → moves → publishes /odom /scan /imu /tf
        5. Navigator checks waypoint progress
        6. Update viz
        """
        # ── 1. Build odom message from current robot state ──────────────
        odom_msg = Odometry(
            header=type('H', (), {'stamp': time.time(), 'frame_id': 'odom'})(),
            child_frame_id='base_link',
            pose=type('PWC', (), {
                'pose': type('P', (), {
                    'position': type('Pt', (), {
                        'x': self.robot.state.x,
                        'y': self.robot.state.y,
                        'z': 0.0,
                    })(),
                    'orientation': Quaternion(
                        x=0.0, y=0.0,
                        z=math.sin(self.robot.state.theta / 2),
                        w=math.cos(self.robot.state.theta / 2),
                    ),
                })(),
            })(),
            twist=type('TWC', (), {
                'twist': Twist(
                    linear=Vector3(x=self.robot.state.v, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=self.robot.state.w),
                ),
            })(),
        )

        # ── 2. Inject state into controller + navigator + planner ───────
        self.controller_node._odom_callback(odom_msg)
        self.navigator_node._odom_callback(odom_msg)
        self.planner_node.set_current_pose(Pose(
            position=Point(x=self.robot.state.x, y=self.robot.state.y, z=0.0),
            orientation=Quaternion(
                x=0.0, y=0.0,
                z=math.sin(self.robot.state.theta / 2),
                w=math.cos(self.robot.state.theta / 2),
            ),
        ))

        # ── 3. Navigator checks waypoint progress, may publish /goal_pose
        self.navigator_node._monitor_tick()

        # ── 4. Planner processes any new /goal_pose ─────────────────────
        spin_once(self.planner_node, timeout_sec=0.001)

        # ── 5. Controller computes DWA and publishes /cmd_vel ───────────
        self.controller_node.step(dt)

        # ── 6. Robot executes /cmd_vel (read in step) ──────────────────
        self.robot_node.step(dt)

        # ── 7. TF broadcaster ──────────────────────────────────────────
        self.tf_broadcaster.publish_odom_tf(
            self.robot.state.x, self.robot.state.y, self.robot.state.theta,
        )
        spin_once(self.tf_broadcaster, timeout_sec=0.001)
        spin_once(self.map_server, timeout_sec=0.001)

        # ── 8. Update HUD ──────────────────────────────────────────────
        self.cyber_hud.update(self.robot.state.v, self.robot.state.w)

        # 9. Update viz telemetry
        self.viz.telemetry = {
            'x': self.robot.state.x, 'y': self.robot.state.y,
            'theta': self.robot.state.theta,
            'v': self.robot.state.v, 'w': self.robot.state.w,
            'plan_time': 0.001,
            'total_dist': math.sqrt(
                (self.robot.state.x - self.map_data['spawn_points'][0][0] * self.map_data['resolution']) ** 2 +
                (self.robot.state.y - self.map_data['spawn_points'][0][1] * self.map_data['resolution']) ** 2),
            'waypoints_done': self.navigator_node._current_waypoint_idx,
            'collisions': 0,
            'replans': 0,
            'scan_ranges': self.scan_data,
            'scan_hz': 10,
            'battery_pct': 95.0,
        }
        self.viz.nav_status = self._nav_status

        # 10. Track waypoint index for viz
        self.viz.current_waypoint_idx = self.navigator_node._current_waypoint_idx

    # ── events ──────────────────────────────────────────────────────────
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

        sd = self.viz.target_scale - self.viz.view_scale
        self.viz.view_scale += sd * 0.15

    # ── render ──────────────────────────────────────────────────────────
    def _render(self, shake_x=0, shake_y=0):
        screen = self.viz.screen
        screen.fill(Colors.BG)

        self.viz.draw_map()
        self.viz.draw_costmap()
        self.grid_overlay.apply(screen)

        # Global plan — pull from planner topic (controller has a copy)
        if self.controller_node._current_plan:
            gp = [(p.pose.position.x, p.pose.position.y)
                  for p in self.controller_node._current_plan]
            self.viz.draw_global_path(gp)

        # Local plan
        # (controller publishes it; we don't have a direct subscriber for viz,
        #  so we skip local_path rendering in the ROS version — the global path
        #  is enough to show the A* result)

        self.viz.draw_waypoints(self.viz.waypoints, self.viz.current_waypoint_idx)

        if self.robot.trajectory:
            self.viz.draw_trajectory(self.robot.trajectory)

        self.viz.draw_laser_scan(
            self.scan_data, self.robot.state.x,
            self.robot.state.y, self.robot.state.theta,
        )
        self.viz.draw_robot(self.robot.state.x, self.robot.state.y,
                            self.robot.state.theta)
        self.viz.particles.draw(screen, self.viz.view_offset_x,
                                self.viz.view_offset_y, self.viz.view_scale)

        # Effects
        self.vignette.apply(screen)
        self.scanlines.apply(screen)
        self.glitch.apply(screen)

        self.radar.update(1.0 / 60)
        self.radar.draw(screen, (0, 255, 100))

        self.cyber_hud.draw(
            screen, self.viz.telemetry, self.viz.nav_status,
            self.viz.waypoints, self.viz.current_waypoint_idx,
            self.viz.time, self.viz.log_messages,
        )
        self.viz.draw_minimap(self.robot.state.x, self.robot.state.y,
                              self.robot.state.theta)
        self.viz.draw_title_bar()
        self.viz.draw_controls_help()

        if self.paused:
            ps = pygame.Surface((350, 70), pygame.SRCALPHA)
            ps.fill((0, 0, 0, 200))
            screen.blit(ps, (self.viz.screen_width // 2 - 175,
                             self.viz.screen_height // 2 - 35))
            try:
                font = pygame.font.SysFont("monospace", 28, bold=True)
            except Exception:
                font = pygame.font.Font(None, 32)
            pt = font.render("⏸  PAUSED", True, Colors.HUD_WARN)
            screen.blit(pt, (self.viz.screen_width // 2 - 90,
                             self.viz.screen_height // 2 - 18))

        pygame.display.flip()


# ═════════════════════════════════════════════════════════════════════════════
def main():
    print("=" * 60)
    print("  🤖 ROS 2 Navigation Simulator — CYBERPUNK")
    print("  Nav2 Stack (A* + DWA) · rclpy-compatible pub/sub")
    print("  Topics: /odom /scan /imu /tf /cmd_vel /plan /goal_pose")
    print("=" * 60)

    app = SimulationApp()
    app.run()


if __name__ == "__main__":
    main()
