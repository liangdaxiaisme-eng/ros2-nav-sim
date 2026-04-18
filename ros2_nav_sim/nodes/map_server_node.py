"""MapServerNode — publishes the map and costmap."""

from __future__ import annotations

import time as _time
from typing import List

import numpy as np

from ros2_nav_sim.ros_compat import Node, QoSProfile, qos_profile_default
from ros2_nav_sim.msgs.std_msgs import Header
from ros2_nav_sim.msgs.geometry_msgs import Point, Quaternion, Pose
from ros2_nav_sim.msgs.nav_msgs import OccupancyGrid, MapMetaData, Costmap


class MapServerNode(Node):
    """Publishes the static map and costmap on ROS 2 topics."""

    def __init__(self, map_data: np.ndarray, resolution: float = 0.05) -> None:
        super().__init__('map_server')

        self.map_data = map_data  # 2D numpy array (0=free, 1=occupied)
        self.resolution = resolution

        # QoS for latched / transient-local
        qos_latched = QoSProfile(
            depth=1,
            durability=QoSProfile.DURABILITY_TRANSIENT_LOCAL,
        )

        # -- Publishers --
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_latched)
        self.costmap_pub = self.create_publisher(Costmap, '/costmap', qos_latched)

        # -- Parameters --
        self.declare_parameter('yaml_filename', 'factory_map.yaml')
        self.declare_parameter('topic_name', '/map')
        self.declare_parameter('frame_id', 'map')

        # Publish immediately on construction
        self._publish_map()

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_map(self) -> None:
        frame_id = self.get_parameter('frame_id').value
        stamp = _time.time()
        height, width = self.map_data.shape

        # Convert to OccupancyGrid: 0=free, 100=occupied, -1=unknown
        occ_data: List[int] = []
        for row in range(height):
            for col in range(width):
                val = self.map_data[row, col]
                if val == 1:
                    occ_data.append(100)
                else:
                    occ_data.append(0)

        grid = OccupancyGrid(
            header=Header(stamp=stamp, frame_id=frame_id),
            info=MapMetaData(
                map_load_time=stamp,
                resolution=self.resolution,
                width=width,
                height=height,
                origin=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
            data=occ_data,
        )
        self.map_pub.publish(grid)
        self.get_logger().info(f'Published map ({width}x{height})')

        # Publish costmap (inflate obstacles)
        costmap_data = self._build_costmap()
        costmap = Costmap(
            header=Header(stamp=stamp, frame_id=frame_id),
            metadata=MapMetaData(
                map_load_time=stamp,
                resolution=self.resolution,
                width=width,
                height=height,
                origin=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
            data=costmap_data,
        )
        self.costmap_pub.publish(costmap)
        self.get_logger().info(f'Published costmap ({width}x{height})')

    # ------------------------------------------------------------------
    # Costmap generation
    # ------------------------------------------------------------------

    def _build_costmap(self, inflation_radius_px: int = 6) -> List[float]:
        """Inflate obstacles into a costmap (0–255)."""
        height, width = self.map_data.shape
        cost = np.zeros((height, width), dtype=np.float64)

        # Mark obstacles
        cost[self.map_data == 1] = 255.0

        # Distance-based inflation
        for dy in range(-inflation_radius_px, inflation_radius_px + 1):
            for dx in range(-inflation_radius_px, inflation_radius_px + 1):
                dist = (dx * dx + dy * dy) ** 0.5
                if dist > inflation_radius_px:
                    continue
                factor = 1.0 - dist / (inflation_radius_px + 1)
                inflated = 200.0 * factor

                src_y0 = max(0, -dy)
                src_y1 = min(height, height - dy)
                src_x0 = max(0, -dx)
                src_x1 = min(width, width - dx)

                dst_y0 = max(0, dy)
                dst_y1 = min(height, height + dy)
                dst_x0 = max(0, dx)
                dst_x1 = min(width, width + dx)

                # Apply where obstacle exists
                patch = self.map_data[src_y0:src_y1, src_x0:src_x1]
                target = cost[dst_y0:dst_y1, dst_x0:dst_x1]
                np.maximum(target, inflated * patch, out=target)

        flat: List[float] = []
        for row in range(height):
            for col in range(width):
                flat.append(float(cost[row, col]))
        return flat
