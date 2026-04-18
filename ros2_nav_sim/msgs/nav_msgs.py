"""ROS 2 message types — nav_msgs."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from .std_msgs import Header
from .geometry_msgs import Pose, PoseStamped, PoseWithCovariance, TwistWithCovariance


@dataclass
class MapMetaData:
    map_load_time: float = 0.0
    resolution: float = 0.05
    width: int = 0
    height: int = 0
    origin: Pose = field(default_factory=Pose)


@dataclass
class OccupancyGrid:
    header: Header = field(default_factory=Header)
    info: MapMetaData = field(default_factory=MapMetaData)
    data: List[int] = field(default_factory=list)


@dataclass
class Odometry:
    header: Header = field(default_factory=Header)
    child_frame_id: str = ""
    pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)
    twist: TwistWithCovariance = field(default_factory=TwistWithCovariance)


@dataclass
class Path:
    header: Header = field(default_factory=Header)
    poses: List[PoseStamped] = field(default_factory=list)


@dataclass
class GridCells:
    header: Header = field(default_factory=Header)
    cell_width: float = 0.0
    cell_height: float = 0.0
    cells: List = field(default_factory=list)  # List[Point]


@dataclass
class Costmap:
    header: Header = field(default_factory=Header)
    metadata: MapMetaData = field(default_factory=MapMetaData)
    data: List[float] = field(default_factory=list)
