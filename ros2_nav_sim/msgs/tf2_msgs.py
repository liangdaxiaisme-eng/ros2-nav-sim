"""ROS 2 message types — tf2_msgs."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from .geometry_msgs import TransformStamped


@dataclass
class TFMessage:
    transforms: List[TransformStamped] = field(default_factory=list)
