"""PlannerNode — wraps AStarPlanner as a ROS 2 node."""

from __future__ import annotations

import math
import time as _time
from typing import List, Optional, Tuple

import numpy as np

from core.path_planner import AStarPlanner

from ros2_nav_sim.ros_compat import Node, qos_profile_default
from ros2_nav_sim.msgs.std_msgs import Header
from ros2_nav_sim.msgs.geometry_msgs import Point, Quaternion, Pose, PoseStamped
from ros2_nav_sim.msgs.nav_msgs import Path, Costmap, MapMetaData


class PlannerNode(Node):
    """Global planner node wrapping AStarPlanner."""

    def __init__(self, costmap: np.ndarray, resolution: float) -> None:
        super().__init__('global_planner')

        self.costmap = costmap
        self.resolution = resolution
        self._planner = AStarPlanner(costmap, resolution)
        self._current_pose: Optional[Pose] = None

        # -- Publishers --
        self.plan_pub = self.create_publisher(Path, '/plan', qos_profile_default)
        self.costmap_pub = self.create_publisher(Costmap, '/global_costmap/costmap', qos_profile_default)

        # -- Subscriber --
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, qos_profile_default,
        )

        # -- Parameters --
        self.declare_parameter('planner_plugin', 'nav2_astar_planner/AStarPlanner')
        self.declare_parameter('tolerance', 0.3)
        self.declare_parameter('use_final_approach_orientation', True)

        # Publish costmap once on startup
        self._publish_costmap()

    # ------------------------------------------------------------------
    # External interface
    # ------------------------------------------------------------------

    def set_current_pose(self, pose: Pose) -> None:
        """Inject the robot's current pose (called externally)."""
        self._current_pose = pose

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, msg: PoseStamped) -> None:
        if self._current_pose is None:
            self.get_logger().warn('No current pose set — ignoring goal')
            return

        t0 = _time.time()

        # Convert world positions → grid cells
        goal_x = int(msg.pose.position.x / self.resolution)
        goal_y = int(msg.pose.position.y / self.resolution)
        start_x = int(self._current_pose.position.x / self.resolution)
        start_y = int(self._current_pose.position.y / self.resolution)

        start = (start_x, start_y)
        goal = (goal_x, goal_y)

        grid_path = self._planner.plan(start, goal)
        elapsed = _time.time() - t0

        if grid_path is None:
            self.get_logger().warn(
                f'Planning failed from {start} to {goal} in {elapsed:.3f}s'
            )
            return

        self.get_logger().info(
            f'Planned path with {len(grid_path)} waypoints in {elapsed:.3f}s'
        )

        # Convert to Path message
        path_msg = self._grid_path_to_msg(grid_path, msg.header.stamp)
        self.plan_pub.publish(path_msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _grid_path_to_msg(self, grid_path: List[Tuple[int, int]], stamp: float) -> Path:
        """Convert a list of grid cells to a Path message."""
        poses: List[PoseStamped] = []
        prev_theta = 0.0

        for idx, (gx, gy) in enumerate(grid_path):
            wx = gx * self.resolution
            wy = gy * self.resolution

            # Compute orientation from segment direction
            if idx < len(grid_path) - 1:
                ngx, ngy = grid_path[idx + 1]
                nwx = ngx * self.resolution
                nwy = ngy * self.resolution
                prev_theta = math.atan2(nwy - wy, nwx - wx)

            poses.append(PoseStamped(
                header=Header(stamp=stamp, frame_id='map'),
                pose=Pose(
                    position=Point(x=wx, y=wy, z=0.0),
                    orientation=Quaternion(
                        x=0.0, y=0.0,
                        z=math.sin(prev_theta / 2.0),
                        w=math.cos(prev_theta / 2.0),
                    ),
                ),
            ))

        return Path(
            header=Header(stamp=stamp, frame_id='map'),
            poses=poses,
        )

    def _publish_costmap(self) -> None:
        h, w = self.costmap.shape
        flat: List[float] = []
        for row in range(h):
            for col in range(w):
                flat.append(float(self.costmap[row, col]))

        msg = Costmap(
            header=Header(stamp=_time.time(), frame_id='map'),
            metadata=MapMetaData(
                resolution=self.resolution,
                width=w,
                height=h,
                origin=Pose(position=Point(x=0.0, y=0.0, z=0.0)),
            ),
            data=flat,
        )
        self.costmap_pub.publish(msg)
