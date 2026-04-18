"""TFBroadcasterNode — manages the TF tree for the simulation."""

from __future__ import annotations

import math
import time as _time

from ros2_nav_sim.ros_compat import Node, QoSProfile, qos_profile_default
from ros2_nav_sim.msgs.std_msgs import Header
from ros2_nav_sim.msgs.geometry_msgs import (
    Quaternion, Vector3, Transform, TransformStamped,
)
from ros2_nav_sim.msgs.tf2_msgs import TFMessage


class TFBroadcasterNode(Node):
    """Publishes static (map→odom) and dynamic (odom→base_link) transforms."""

    def __init__(self) -> None:
        super().__init__('tf_broadcaster')

        # -- Publishers --
        self.tf_pub = self.create_publisher(TFMessage, '/tf', qos_profile_default)
        self.tf_static_pub = self.create_publisher(
            TFMessage, '/tf_static',
            QoSProfile(depth=1, durability=QoSProfile.DURABILITY_TRANSIENT_LOCAL),
        )

        # -- Parameters --
        self.declare_parameter('publish_rate', 50.0)

        # Publish the static transform: map → odom (identity)
        self._publish_static_tf()

        # Timer for periodic TF publishing (map→odom is static but we re-publish for safety)
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / max(rate, 1.0), self._timer_tick)

        # Odom→base_link state (updated externally)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0

    # ------------------------------------------------------------------
    # External interface
    # ------------------------------------------------------------------

    def publish_odom_tf(self, x: float, y: float, theta: float) -> None:
        """Update the odom → base_link transform from robot state."""
        self._odom_x = x
        self._odom_y = y
        self._odom_theta = theta

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _publish_static_tf(self) -> None:
        """Publish map → odom as an identity (static) transform."""
        stamp = _time.time()
        static_tf = TransformStamped(
            header=Header(stamp=stamp, frame_id='map'),
            child_frame_id='odom',
            transform=Transform(
                translation=Vector3(x=0.0, y=0.0, z=0.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        msg = TFMessage(transforms=[static_tf])
        self.tf_static_pub.publish(msg)
        self.get_logger().info('Published static TF: map → odom')

    def _timer_tick(self) -> None:
        """Periodically publish the odom → base_link dynamic transform."""
        stamp = _time.time()
        odom_tf = TransformStamped(
            header=Header(stamp=stamp, frame_id='odom'),
            child_frame_id='base_link',
            transform=Transform(
                translation=Vector3(x=self._odom_x, y=self._odom_y, z=0.0),
                rotation=Quaternion(
                    x=0.0, y=0.0,
                    z=math.sin(self._odom_theta / 2.0),
                    w=math.cos(self._odom_theta / 2.0),
                ),
            ),
        )
        msg = TFMessage(transforms=[odom_tf])
        self.tf_pub.publish(msg)
