"""
无头渲染模式 — 生成演示视频
无需显示器，使用虚拟帧缓冲渲染并输出 MP4
"""
import sys
import os
import math
import time
import subprocess
import numpy as np

# SDL 无头模式
os.environ['SDL_VIDEODRIVER'] = 'dummy'
os.environ['SDL_AUDIODRIVER'] = 'dummy'

import pygame
pygame.init()
# 创建最小显示表面用于初始化
pygame.display.set_mode((1, 1))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maps.complex_map import generate_complex_map
from core.path_planner import AStarPlanner, DWAPlanner, Pose, Velocity
from core.robot import DifferentialDriveRobot
from core.visualizer import NavigationVisualizer, Colors
from core.navigator import WaypointNavigator, NavState


def world_to_surface_array(surface):
    """pygame surface → numpy array"""
    w, h = surface.get_size()
    arr = pygame.surfarray.array3d(surface)
    # (w, h, 3) → (h, w, 3)  旋转
    arr = np.transpose(arr, (1, 0, 2))
    return arr


def generate_video(output_path="navigation_demo.mp4", duration=60, fps=30):
    """生成演示视频"""
    print(f"🎬 Generating video: {output_path} ({duration}s @ {fps}fps)")

    WIDTH, HEIGHT = 1280, 720

    # 初始化
    viz = NavigationVisualizer(WIDTH, HEIGHT)

    # 生成地图
    print("   🏗️  Generating map...")
    map_data = generate_complex_map(seed=42)

    viz.load_map(map_data)
    viz.show_costmap = False
    viz.show_laser = True
    viz.show_trajectory = True

    # 规划器
    costmap = map_data['costmap']
    resolution = map_data['resolution']
    obstacle_mask = np.isin(map_data['grid'], [1, 2, 3, 5])

    a_star = AStarPlanner(costmap, resolution)
    dwa = DWAPlanner(costmap, resolution)

    # 机器人
    spawn = map_data['spawn_points'][0] if map_data['spawn_points'] else (100, 100)
    robot = DifferentialDriveRobot(
        spawn[0] * resolution,
        spawn[1] * resolution,
        theta=math.pi / 4
    )

    # 航点 — 复杂路线
    waypoints = [
        (50 * resolution, 30 * resolution),
        (30 * resolution, 67 * resolution),
        (30 * resolution, 100 * resolution),
        (65 * resolution, 100 * resolution),
        (65 * resolution, 67 * resolution),
        (100 * resolution, 67 * resolution),
        (100 * resolution, 100 * resolution),
        (150 * resolution, 100 * resolution),
        (150 * resolution, 160 * resolution),
        (100 * resolution, 160 * resolution),
        (50 * resolution, 160 * resolution),
    ]

    # 导航器
    navigator = WaypointNavigator(a_star, dwa, robot, resolution)
    navigator.set_waypoints(waypoints)
    navigator.start()

    viz.waypoints = waypoints
    viz.planning_algorithm = "A* + DWA"

    # 视图参数 — 自动适配
    map_w = map_data['width'] * resolution
    map_h = map_data['height'] * resolution
    viz.view_scale = min(WIDTH / map_w, HEIGHT / map_h) * 0.75
    viz.target_scale = viz.view_scale
    viz.view_offset_x = 30
    viz.view_offset_y = 25

    # FFmpeg 进程
    ffmpeg_cmd = [
        'ffmpeg', '-y',
        '-f', 'rawvideo',
        '-vcodec', 'rawvideo',
        '-s', f'{WIDTH}x{HEIGHT}',
        '-pix_fmt', 'rgb24',
        '-r', str(fps),
        '-i', '-',
        '-c:v', 'libx264',
        '-preset', 'medium',
        '-crf', '20',
        '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart',
        output_path
    ]

    proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    total_frames = duration * fps
    sim_dt = 1.0 / fps
    scan_data = []
    scan_timer = 0.0

    for frame_idx in range(total_frames):
        if frame_idx % 100 == 0:
            progress = frame_idx / total_frames * 100
            wp_status = f"WP {navigator.current_waypoint_idx + 1}/{len(waypoints)}"
            print(f"   Frame {frame_idx}/{total_frames} ({progress:.0f}%) — {wp_status}")

        # 更新导航
        if navigator.state == NavState.PLANNING:
            navigator._plan_to_current_waypoint()
            if navigator.state == NavState.NAVIGATING:
                viz.global_path = navigator.global_path

        elif navigator.state == NavState.NAVIGATING:
            current_pos = (robot.state.x, robot.state.y)
            target = navigator._get_current_target(current_pos)

            if target is None:
                navigator.current_waypoint_idx += 1
                viz.current_waypoint_idx = navigator.current_waypoint_idx
                viz.add_log(f"Waypoint {navigator.current_waypoint_idx} reached!")
                viz.particles.emit(robot.state.x, robot.state.y,
                                   count=30, color=(100, 255, 150),
                                   spread=3.0, speed=40.0, lifetime=1.0)

                if navigator.current_waypoint_idx >= len(waypoints):
                    navigator.state = NavState.ARRIVED
                    viz.nav_status = "ARRIVED"
                    viz.add_log("🎉 All waypoints reached!")
                    viz.particles.emit(robot.state.x, robot.state.y,
                                       count=80, color=(255, 200, 50),
                                       spread=5.0, speed=60.0, lifetime=1.5)
                else:
                    navigator.state = NavState.PLANNING
            else:
                goal_pose = Pose(target[0], target[1], 0.0)
                current_pose = Pose(robot.state.x, robot.state.y, robot.state.theta)
                current_vel = Velocity(robot.state.v, robot.state.w)

                vel_cmd, local_traj = dwa.plan(current_pose, goal_pose, current_vel)
                robot.update(vel_cmd.v, vel_cmd.w, sim_dt)

                if navigator.last_positions:
                    last = navigator.last_positions[-1]
                    dist = math.sqrt(
                        (current_pos[0] - last[0]) ** 2 +
                        (current_pos[1] - last[1]) ** 2
                    )
                    navigator.total_distance += dist
                navigator.last_positions.append(current_pos)
                if len(navigator.last_positions) > 100:
                    navigator.last_positions.pop(0)

                if local_traj:
                    viz.local_path = [(p.x, p.y) for p in local_traj]

        elif navigator.state == NavState.REPLAN:
            navigator._plan_to_current_waypoint()
            if navigator.state == NavState.NAVIGATING:
                viz.add_log(f"Replanned (#{navigator.replan_count})")
                viz.global_path = navigator.global_path

        elif navigator.state == NavState.ARRIVED:
            pass

        # 传感器
        scan_timer += sim_dt
        if scan_timer >= 0.1:
            scan_timer = 0.0
            scan = robot.get_laser_scan(obstacle_mask, resolution, num_rays=180, max_range=8.0)
            scan_data = scan.ranges

        # 粒子
        viz.particles.update(sim_dt)

        # 遥测
        viz.telemetry = {
            'x': robot.state.x, 'y': robot.state.y, 'theta': robot.state.theta,
            'v': robot.state.v, 'w': robot.state.w,
            'plan_time': 0.001, 'total_dist': navigator.total_distance,
            'waypoints_done': navigator.current_waypoint_idx,
            'collisions': navigator.collision_count,
            'replans': navigator.replan_count,
            'scan_ranges': scan_data, 'scan_hz': 10,
        }
        viz.nav_status = navigator.state.value

        # 渲染
        screen = viz.screen
        screen.fill(Colors.BG)

        viz.draw_map()
        viz.draw_costmap()

        if viz.global_path:
            viz.draw_global_path(viz.global_path)
        if viz.local_path:
            viz.draw_local_path(viz.local_path)

        viz.draw_waypoints(viz.waypoints, viz.current_waypoint_idx)

        if robot.trajectory:
            viz.draw_trajectory(robot.trajectory)

        viz.draw_laser_scan(scan_data, robot.state.x, robot.state.y, robot.state.theta)
        viz.draw_robot(robot.state.x, robot.state.y, robot.state.theta)

        viz.particles.draw(screen, viz.view_offset_x, viz.view_offset_y, viz.view_scale)
        viz.draw_hud(viz.telemetry)
        viz.draw_minimap(robot.state.x, robot.state.y, robot.state.theta)
        viz.draw_title_bar()
        viz.draw_controls_help()

        viz.time += sim_dt
        viz.frame_count += 1

        # 输出帧
        frame_data = world_to_surface_array(screen)
        proc.stdin.write(frame_data.tobytes())

    proc.stdin.close()
    proc.wait()

    print(f"   ✅ Video saved: {output_path}")
    size_mb = os.path.getsize(output_path) / (1024 * 1024)
    print(f"   📦 Size: {size_mb:.1f} MB")

    return output_path


if __name__ == "__main__":
    output = sys.argv[1] if len(sys.argv) > 1 else "/root/.openclaw/workspace/ros2-nav-sim/navigation_demo.mp4"
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 45
    generate_video(output, duration=duration, fps=30)
