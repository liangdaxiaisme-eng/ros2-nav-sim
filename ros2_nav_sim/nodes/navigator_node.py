"""NavigatorNode — NavigateToPose action server node."""

from __future__ import annotations

import math
import time as _time
from typing import List, Optional, Tuple

from ros2_nav_sim.ros_compat import Node, qos_profile_default
from ros2_nav_sim.msgs.std_msgs import Header, String
from ros2_nav_sim.msgs.geometry_msgs import (
    Point, Quaternion, Pose, PoseStamped,
)
from ros2_nav_sim.msgs.nav_msgs import Odometry, Path
from ros2_nav_sim.actions.navigate_to_pose import (
    NavigateToPose, GoalHandle,
    ActionServer as NavigateActionServer,
)


class NavigatorNode(Node):
    """Action-server node that orchestrates planning + control for NavigateToPose."""

    def __init__(self, planner_node, controller_node) -> None:
        super().__init__('waypoint_navigator')

        self._planner = planner_node
        self._controller = controller_node

        # Navigation state
        self._waypoints: List[Tuple[float, float]] = []
        self._current_waypoint_idx = 0
        self._navigating = False
        self._start_time = 0.0
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_theta: float = 0.0

        # -- Action server --
        self._action_server = NavigateActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            self._navigate_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        # -- Subscribers --
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, qos_profile_default,
        )

        # -- Publishers --
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', qos_profile_default)
        self.status_pub = self.create_publisher(String, '/navigation_status', qos_profile_default)

        # -- Parameters --
        self.declare_parameter('waypoint_task_executor_plugin', '')
        self.declare_parameter('navigate_to_pose_action_server_name', 'navigate_to_pose')

        # Timer for patrol / navigation monitoring (disabled — driven externally)
        # self.create_timer(0.1, self._monitor_tick)

    # ------------------------------------------------------------------
    # External interface
    # ------------------------------------------------------------------

    def set_waypoints(self, waypoints: List[Tuple[float, float]]) -> None:
        """Set a list of (x, y) waypoints for patrol."""
        self._waypoints = list(waypoints)
        self._current_waypoint_idx = 0
        self.get_logger().info(f'Set {len(waypoints)} waypoints')

    def start_patrol(self) -> None:
        """Start the waypoint patrol loop."""
        if not self._waypoints:
            self.get_logger().warn('No waypoints set for patrol')
            return
        self._navigating = True
        self._start_time = _time.time()
        self._current_waypoint_idx = 0
        self._send_waypoint(self._waypoints[0])
        self.get_logger().info('Patrol started')

    # ------------------------------------------------------------------
    # Odom callback
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._odom_x = pose.position.x
        self._odom_y = pose.position.y
        self._odom_theta = _quat_to_yaw(pose.orientation)

        # Also feed current pose to planner
        self._planner.set_current_pose(pose)

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal: NavigateToPose.Goal) -> bool:
        self.get_logger().info(
            f'Goal received: ({goal.pose.pose.position.x:.2f}, '
            f'{goal.pose.pose.position.y:.2f})'
        )
        return True

    def _cancel_callback(self, goal_handle: GoalHandle) -> int:
        self.get_logger().info('Cancel requested')
        goal_handle.request_cancel()
        self._navigating = False
        return 0  # CANCEL_ACCEPTED

    def _navigate_callback(self, goal_handle: GoalHandle) -> NavigateToPose.Result:
        """Execute the NavigateToPose action."""
        goal = goal_handle.goal
        t_start = _time.time()

        # Publish goal to trigger global planner
        self.goal_pub.publish(goal.pose)
        self._publish_status('PLANNING')

        goal_x = goal.pose.pose.position.x
        goal_y = goal.pose.pose.position.y
        tolerance = 0.3

        # Monitor loop
        while True:
            if goal_handle.is_cancel_requested():
                self._publish_status('CANCELLED')
                return NavigateToPose.Result(error_code=0, error_msg='cancelled')

            dx = goal_x - self._odom_x
            dy = goal_y - self._odom_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < tolerance:
                self._publish_status('ARRIVED')
                self.get_logger().info(f'Goal reached in {_time.time() - t_start:.1f}s')
                return NavigateToPose.Result(error_code=0, error_msg='')

            # Publish feedback
            elapsed = _time.time() - t_start
            feedback = NavigateToPose.Feedback(
                current_pose=PoseStamped(
                    header=Header(stamp=_time.time(), frame_id='map'),
                    pose=Pose(
                        position=Point(x=self._odom_x, y=self._odom_y, z=0.0),
                        orientation=Quaternion(
                            x=0.0, y=0.0,
                            z=math.sin(self._odom_theta / 2.0),
                            w=math.cos(self._odom_theta / 2.0),
                        ),
                    ),
                ),
                navigation_time=elapsed,
                estimated_time_remaining=dist / max(0.1, abs(self._odom_x)),
                distance_remaining=dist,
            )
            goal_handle.publish_feedback(feedback)
            self._publish_status('NAVIGATING')

            _time.sleep(0.05)

    # ------------------------------------------------------------------
    # Patrol monitoring
    # ------------------------------------------------------------------

    def _monitor_tick(self) -> None:
        """Periodic check for waypoint patrol progress."""
        if not self._navigating or not self._waypoints:
            return

        if self._current_waypoint_idx >= len(self._waypoints):
            self._navigating = False
            self._publish_status('PATROL_COMPLETE')
            self.get_logger().info('Patrol complete!')
            return

        # Check if we've reached the current waypoint
        target_x, target_y = self._waypoints[self._current_waypoint_idx]
        dx = target_x - self._odom_x
        dy = target_y - self._odom_y
        dist = math.sqrt(dx * dx + dy * dy)

        tolerance = 0.5
        if dist < tolerance:
            self._current_waypoint_idx += 1
            if self._current_waypoint_idx < len(self._waypoints):
                self._send_waypoint(self._waypoints[self._current_waypoint_idx])

    def _send_waypoint(self, waypoint: Tuple[float, float]) -> None:
        """Publish a waypoint as a PoseStamped goal."""
        wx, wy = waypoint
        goal = PoseStamped(
            header=Header(stamp=_time.time(), frame_id='map'),
            pose=Pose(
                position=Point(x=wx, y=wy, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'Waypoint {self._current_waypoint_idx + 1}/{len(self._waypoints)}: '
            f'({wx:.2f}, {wy:.2f})'
        )

    def _publish_status(self, status: str) -> None:
        self.status_pub.publish(String(data=status))


def _quat_to_yaw(q: Quaternion) -> float:
    """Extract yaw from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
