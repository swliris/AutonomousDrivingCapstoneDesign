from dataclasses import dataclass, field
from enum import Enum
import math
import numpy as np


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose:
    x: float = 0.
    y: float = 0.
    yaw: float = 0.

    def vector(self):
        return np.array([self.x, self.y, self.yaw])


@dataclass(frozen=True)
class Vehicle:
    wheelbase: float
    width: float
    front: float  # rear axle -> foremost body/sensor/jig point
    rear: float   # rear axle -> rearmost body/sensor/jig point
    track: float
    tire_length: float
    tire_width: float
    max_steer: float
    margin: float = .06


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float = 0.
    direction: int = 1


class Light(str, Enum):
    UNKNOWN = "unknown"
    RED = "red"
    GREEN = "green"
    YELLOW = "yellow"


@dataclass
class Observation:
    stamp: float
    front_stamp: float
    rear_stamp: float
    lidar_stamp: float
    pose: Pose
    localized: bool
    speed: float = 0.
    light: Light = Light.UNKNOWN
    light_stamp: float = -math.inf
    lanes: dict[int, list[Waypoint]] = field(default_factory=dict)
    obstacles: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    # A returned scan alone does NOT establish free space in every direction.
    scan_angles: np.ndarray = field(default_factory=lambda: np.empty(0))
    scan_ranges: np.ndarray = field(default_factory=lambda: np.empty(0))
    stop_zone_distance: float | None = None
    in_stop_zone: bool = False
    gate: bool = False
    parallel_entry: bool = False
    out_contact: bool = False
    parked_valid: bool = False
    parked_slot: str | None = None
    parking_line_contact: bool = False
    outside: bool = False
    collision: bool = False
    pose_stamp: float | None = None
    feedback_stamp: float | None = None


@dataclass
class Command:
    speed: float = 0.  # signed target m/s (converted by measured lookup)
    steer: float = 0.  # road wheel radians; positive = left
    reason: str = "stop"
    state: str = "idle"
    path: list[Waypoint] = field(default_factory=list)
