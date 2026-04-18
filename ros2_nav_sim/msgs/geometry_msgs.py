"""ROS 2 message types — geometry_msgs."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from .std_msgs import Header


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Pose:
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class PoseStamped:
    header: Header = field(default_factory=Header)
    pose: Pose = field(default_factory=Pose)


@dataclass
class PoseWithCovariance:
    pose: Pose = field(default_factory=Pose)
    covariance: List[float] = field(default_factory=lambda: [0.0] * 36)


@dataclass
class Twist:
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)


@dataclass
class TwistWithCovariance:
    twist: Twist = field(default_factory=Twist)
    covariance: List[float] = field(default_factory=lambda: [0.0] * 36)


@dataclass
class Transform:
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)


@dataclass
class TransformStamped:
    header: Header = field(default_factory=Header)
    child_frame_id: str = ""
    transform: Transform = field(default_factory=Transform)


@dataclass
class Accel:
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)
