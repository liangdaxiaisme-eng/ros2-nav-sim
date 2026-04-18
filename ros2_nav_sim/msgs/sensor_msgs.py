"""ROS 2 message types — sensor_msgs."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List

from .std_msgs import Header
from .geometry_msgs import Quaternion, Vector3


@dataclass
class LaserScan:
    header: Header = field(default_factory=Header)
    angle_min: float = 0.0
    angle_max: float = 2.0 * math.pi
    angle_increment: float = math.pi / 180.0
    time_increment: float = 0.0
    scan_time: float = 0.0
    range_min: float = 0.1
    range_max: float = 10.0
    ranges: List[float] = field(default_factory=list)
    intensities: List[float] = field(default_factory=list)


@dataclass
class Imu:
    header: Header = field(default_factory=Header)
    orientation: Quaternion = field(default_factory=Quaternion)
    orientation_covariance: List[float] = field(default_factory=lambda: [0.0] * 9)
    angular_velocity: Vector3 = field(default_factory=Vector3)
    angular_velocity_covariance: List[float] = field(default_factory=lambda: [0.0] * 9)
    linear_acceleration: Vector3 = field(default_factory=Vector3)
    linear_acceleration_covariance: List[float] = field(default_factory=lambda: [0.0] * 9)


@dataclass
class BatteryState:
    header: Header = field(default_factory=Header)
    voltage: float = 12.6
    temperature: float = 25.0
    current: float = 0.0
    charge: float = 0.0
    capacity: float = 10.0
    design_capacity: float = 10.0
    percentage: float = 1.0
    power_supply_status: int = 0
    power_supply_health: int = 0
    power_supply_technology: int = 0
    present: bool = True
