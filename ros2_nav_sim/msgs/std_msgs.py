"""ROS 2 message types — std_msgs."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class Header:
    stamp: float = 0.0
    frame_id: str = ""


@dataclass
class String:
    data: str = ""


@dataclass
class Bool:
    data: bool = False


@dataclass
class Float32:
    data: float = 0.0


@dataclass
class Float64:
    data: float = 0.0


@dataclass
class Int32:
    data: int = 0
