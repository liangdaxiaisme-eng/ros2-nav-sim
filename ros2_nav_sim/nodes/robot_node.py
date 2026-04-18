"""RobotNode — wraps DifferentialDriveRobot as a ROS 2 node."""

from __future__ import annotations

import math
import time as _time
from typing import Tuple

from ros2_nav_sim.ros_compat import (
    Node, QoSProfile, Time,
    qos_profile_default, qos_profile_sensor_data,
)
from ros2_nav_sim.msgs.std_msgs import Header
from ros2_nav_sim.msgs.geometry_msgs import (
    Point, Quaternion, Pose, PoseWithCovariance, Twist, TwistWithCovariance,
    Vector3, Transform, TransformStamped,
)
from ros2_nav_sim.msgs.sensor_msgs import LaserScan as LaserScanMsg, Imu, BatteryState
from ros2_nav_sim.msgs.nav_msgs import Odometry, PoseWithCovariance as _unused  # ensure loaded
from ros2_nav_sim.msgs.tf2_msgs import TFMessage


class RobotNode(Node):
    """ROS 2 node wrapping a DifferentialDriveRobot."""

    def __init__(self, robot, obstacle_mask, resolution: float) -> None:
        super().__init__('robot_state_publisher')

        self.robot = robot
        self.obstacle_mask = obstacle_mask
        self.resolution = resolution

        # -- Publishers --
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile_default)
        self.scan_pub = self.create_publisher(LaserScanMsg, '/scan', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_status', qos_profile_default)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', qos_profile_default)

        # -- Subscriber --
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, qos_profile_default,
        )

        # -- Parameters --
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('scan_range_max', 8.0)
        self.declare_parameter('scan_num_rays', 180)
        self.declare_parameter('publish_rate', 20.0)

        # -- Timer for publishing at fixed rate (disabled — driven externally by main loop) --
        # self.create_timer(1.0 / 20.0, self._publish_tick)

        # -- State --
        self._last_cmd_vel: Tuple[float, float] = (0.0, 0.0)
        self._frame_count = 0
        self._battery_charge = 10.0  # Ah remaining
        self._battery_capacity = 10.0

    # ------------------------------------------------------------------
    # External interface
    # ------------------------------------------------------------------

    def step(self, dt: float) -> None:
        """Advance simulation by *dt* seconds (called externally by main loop)."""
        v, w = self._last_cmd_vel
        self.robot.update(v, w, dt)
        self._frame_count += 1
        self._publish_all(dt)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._last_cmd_vel = (msg.linear.x, msg.angular.z)

    def _publish_tick(self) -> None:
        """Timer-driven publish at the configured rate."""
        dt = 1.0 / max(self.get_parameter('publish_rate').value, 1.0)
        self.step(dt)

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _now(self) -> Time:
        return self.get_clock().now()

    def _stamp(self) -> float:
        return _time.time()

    def _publish_all(self, dt: float) -> None:
        stamp = self._stamp()
        base_frame_id = self.get_parameter('base_frame_id').value
        odom_frame_id = self.get_parameter('odom_frame_id').value

        # 1. Odometry
        self._publish_odometry(stamp, odom_frame_id, base_frame_id)

        # 2. LaserScan (every 5 ticks → ~10 Hz at 20 Hz publish rate)
        if self._frame_count % 5 == 0:
            self._publish_scan(stamp, base_frame_id)

        # 3. IMU
        self._publish_imu(stamp, base_frame_id)

        # 4. TF (odom → base_link)
        self._publish_tf(stamp, odom_frame_id, base_frame_id)

        # 5. Battery
        self._publish_battery(stamp, base_frame_id)

    # ---- individual publishers ----

    def _publish_odometry(self, stamp: float, odom_frame: str, child_frame: str) -> None:
        odom = self.robot.get_odometry()
        msg = Odometry(
            header=Header(stamp=stamp, frame_id=odom_frame),
            child_frame_id=child_frame,
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=odom.x, y=odom.y, z=0.0),
                    orientation=Quaternion(
                        x=0.0, y=0.0,
                        z=math.sin(odom.theta / 2.0),
                        w=math.cos(odom.theta / 2.0),
                    ),
                ),
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=odom.v, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=odom.w),
                ),
            ),
        )
        self.odom_pub.publish(msg)

    def _publish_scan(self, stamp: float, frame_id: str) -> None:
        num_rays = self.get_parameter('scan_num_rays').value
        max_range = self.get_parameter('scan_range_max').value
        scan_data = self.robot.get_laser_scan(
            self.obstacle_mask, self.resolution,
            num_rays=num_rays, max_range=max_range,
        )
        msg = LaserScanMsg(
            header=Header(stamp=stamp, frame_id=frame_id),
            angle_min=scan_data.angle_min,
            angle_max=scan_data.angle_max,
            angle_increment=scan_data.angle_increment,
            range_min=scan_data.range_min,
            range_max=scan_data.range_max,
            ranges=list(scan_data.ranges),
            intensities=list(scan_data.intensities),
        )
        self.scan_pub.publish(msg)

    def _publish_imu(self, stamp: float, frame_id: str) -> None:
        imu_data = self.robot.get_imu_data()
        msg = Imu(
            header=Header(stamp=stamp, frame_id=frame_id),
            orientation=Quaternion(
                x=imu_data['orientation']['x'],
                y=imu_data['orientation']['y'],
                z=imu_data['orientation']['z'],
                w=imu_data['orientation']['w'],
            ),
            angular_velocity=Vector3(
                x=imu_data['angular_velocity']['x'],
                y=imu_data['angular_velocity']['y'],
                z=imu_data['angular_velocity']['z'],
            ),
            linear_acceleration=Vector3(
                x=imu_data['linear_acceleration']['x'],
                y=imu_data['linear_acceleration']['y'],
                z=imu_data['linear_acceleration']['z'],
            ),
        )
        self.imu_pub.publish(msg)

    def _publish_tf(self, stamp: float, parent: str, child: str) -> None:
        odom = self.robot.get_odometry()
        tf_stamped = TransformStamped(
            header=Header(stamp=stamp, frame_id=parent),
            child_frame_id=child,
            transform=Transform(
                translation=Vector3(x=odom.x, y=odom.y, z=0.0),
                rotation=Quaternion(
                    x=0.0, y=0.0,
                    z=math.sin(odom.theta / 2.0),
                    w=math.cos(odom.theta / 2.0),
                ),
            ),
        )
        msg = TFMessage(transforms=[tf_stamped])
        self.tf_pub.publish(msg)

    def _publish_battery(self, stamp: float, frame_id: str) -> None:
        # Simple drain model: ~1% per second of operation
        drain = 0.01 / 60.0  # per tick at 20 Hz
        self._battery_charge = max(0.0, self._battery_charge - drain)
        pct = self._battery_charge / self._battery_capacity

        msg = BatteryState(
            header=Header(stamp=stamp, frame_id=frame_id),
            voltage=12.6 * pct,
            temperature=25.0,
            current=2.0 if self.robot.state.v != 0.0 or self.robot.state.w != 0.0 else 0.1,
            charge=self._battery_charge,
            capacity=self._battery_capacity,
            design_capacity=10.0,
            percentage=pct,
            power_supply_status=2 if pct > 0.1 else 3,  # discharging / charging
            power_supply_health=0,
            power_supply_technology=2,  # lithium-ion
            present=True,
        )
        self.battery_pub.publish(msg)
