"""ControllerNode — wraps DWAPlanner as a ROS 2 node."""

from __future__ import annotations

import math
import time as _time
from typing import List, Optional, Tuple

import numpy as np

from core.path_planner import DWAPlanner, Pose as PlannerPose, Velocity as PlannerVelocity

from ros2_nav_sim.ros_compat import Node, qos_profile_default
from ros2_nav_sim.msgs.std_msgs import Header
from ros2_nav_sim.msgs.geometry_msgs import (
    Point, Quaternion, Pose, PoseStamped, Twist, Vector3,
)
from ros2_nav_sim.msgs.nav_msgs import Odometry, Path


class ControllerNode(Node):
    """Local planner / controller node wrapping DWAPlanner."""

    def __init__(self, costmap: np.ndarray, resolution: float) -> None:
        super().__init__('controller_server')

        self.costmap = costmap
        self.resolution = resolution
        self._planner = DWAPlanner(costmap, resolution)

        # State from subscriptions
        self._current_plan: Optional[List[PoseStamped]] = None
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_theta: float = 0.0
        self._odom_v: float = 0.0
        self._odom_w: float = 0.0

        # -- Publishers --
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile_default)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', qos_profile_default)

        # -- Subscribers --
        self.plan_sub = self.create_subscription(
            Path, '/plan', self._plan_callback, qos_profile_default,
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, qos_profile_default,
        )

        # -- Parameters --
        self.declare_parameter('FollowPath.plugin', 'dwb_core::DWBLocalPlanner')
        self.declare_parameter('max_vel_x', 1.0)
        self.declare_parameter('min_vel_x', -0.2)
        self.declare_parameter('max_vel_theta', 2.0)
        self.declare_parameter('acc_lim_x', 0.5)
        self.declare_parameter('acc_lim_theta', 3.0)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('frequency', 10.0)

        # Timer for periodic compute (disabled — driven externally by main loop)
        # self.create_timer(1.0 / max(self.get_parameter('frequency').value, 1.0),
        #                   lambda: self.compute_velocity_commands())
        self._last_cmd_time = _time.time()

    # ------------------------------------------------------------------
    # External interface
    # ------------------------------------------------------------------

    def step(self, dt: float) -> None:
        """Advance controller by *dt* seconds (called externally by main loop)."""
        self.compute_velocity_commands()

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _plan_callback(self, msg: Path) -> None:
        self._current_plan = list(msg.poses)
        self.get_logger().info(f'Received global plan with {len(msg.poses)} poses')

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._odom_x = pose.position.x
        self._odom_y = pose.position.y
        self._odom_theta = _quat_to_yaw(pose.orientation)
        self._odom_v = msg.twist.twist.linear.x
        self._odom_w = msg.twist.twist.angular.z

    # ------------------------------------------------------------------
    # Compute velocity
    # ------------------------------------------------------------------

    def compute_velocity_commands(self) -> Optional[Twist]:
        """Compute and publish velocity commands based on current state."""
        if self._current_plan is None or len(self._current_plan) == 0:
            # No plan — publish zero velocity
            zero = Twist()
            self.cmd_pub.publish(zero)
            return zero

        # Pick a lookahead target from the plan
        target_pose_stamped = self._select_target()
        if target_pose_stamped is None:
            zero = Twist()
            self.cmd_pub.publish(zero)
            return zero

        # Build planner inputs
        current = PlannerPose(
            x=self._odom_x,
            y=self._odom_y,
            theta=self._odom_theta,
        )
        goal = PlannerPose(
            x=target_pose_stamped.pose.position.x,
            y=target_pose_stamped.pose.position.y,
            theta=_quat_to_yaw(target_pose_stamped.pose.orientation),
        )
        vel = PlannerVelocity(v=self._odom_v, w=self._odom_w)

        # DWA plan
        best_vel, trajectory = self._planner.plan(current, goal, vel)

        # Publish cmd_vel
        cmd = Twist(
            linear=Vector3(x=best_vel.v, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=best_vel.w),
        )
        self.cmd_pub.publish(cmd)

        # Publish local plan visualization
        if trajectory:
            stamp = _time.time()
            poses = [
                PoseStamped(
                    header=Header(stamp=stamp, frame_id='map'),
                    pose=Pose(
                        position=Point(x=p.x, y=p.y, z=0.0),
                        orientation=Quaternion(
                            x=0.0, y=0.0,
                            z=math.sin(p.theta / 2.0),
                            w=math.cos(p.theta / 2.0),
                        ),
                    ),
                )
                for p in trajectory
            ]
            local_path = Path(
                header=Header(stamp=stamp, frame_id='map'),
                poses=poses,
            )
            self.local_plan_pub.publish(local_path)

        return cmd

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _select_target(self) -> Optional[PoseStamped]:
        """Pick a lookahead target from the global plan."""
        tolerance = self.get_parameter('goal_tolerance').value

        # Find closest point on plan
        min_dist = float('inf')
        closest_idx = 0
        for idx, ps in enumerate(self._current_plan):
            dx = ps.pose.position.x - self._odom_x
            dy = ps.pose.position.y - self._odom_y
            d = math.sqrt(dx * dx + dy * dy)
            if d < min_dist:
                min_dist = d
                closest_idx = idx

        # Look ahead by distance (2m), not fixed number of points
        # Grid waypoints are ~0.035m apart, so 2m ≈ 57 points
        lookahead_dist = 2.0  # meters
        target_idx = closest_idx
        for idx in range(closest_idx, len(self._current_plan)):
            ps = self._current_plan[idx]
            dx = ps.pose.position.x - self._odom_x
            dy = ps.pose.position.y - self._odom_y
            d = math.sqrt(dx * dx + dy * dy)
            if d >= lookahead_dist:
                target_idx = idx
                break
        else:
            target_idx = len(self._current_plan) - 1

        return self._current_plan[target_idx]


def _quat_to_yaw(q: Quaternion) -> float:
    """Extract yaw from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
