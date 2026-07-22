"""Pure SI/REP-103 conversions used at the ROS boundary only."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from components.radar_driver import Pose2D, RadarPoint


@dataclass(frozen=True, slots=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True, slots=True)
class RosPose2D:
    x_m: float
    y_m: float
    yaw_rad: float


def normalize_yaw_rad(yaw_rad: float) -> float:
    return (float(yaw_rad) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    half = float(yaw_rad) / 2.0
    return Quaternion(0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(quaternion: Quaternion) -> float:
    return normalize_yaw_rad(math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    ))


def radar_pose_to_ros(pose: Pose2D) -> RosPose2D:
    """cm/CW radar pose -> m/CCW REP-103 pose."""

    return RosPose2D(pose.x_cm / 100.0, pose.y_cm / 100.0, math.radians(-pose.yaw_cw_deg))


def ros_pose_to_radar(pose: RosPose2D) -> Pose2D:
    return Pose2D(pose.x_m * 100.0, pose.y_m * 100.0, -math.degrees(pose.yaw_rad))


def radar_points_to_scan_ranges(
    points: Iterable[RadarPoint], *, bins: int = 720, range_max_m: float = 12.0
) -> tuple[float, ...]:
    """Place D500 clockwise points into CCW ROS LaserScan bins in metres."""

    if bins < 2 or range_max_m <= 0.0:
        raise ValueError("bins must be >= 2 and range_max_m must be positive")
    ranges = [math.inf] * bins
    for point in points:
        distance_m = point.distance_mm / 1000.0
        if distance_m <= 0.0 or distance_m > range_max_m:
            continue
        ros_angle = -math.radians(point.angle_cw_deg)
        normalized = (ros_angle + math.pi) % (2.0 * math.pi)
        index = min(bins - 1, int(normalized / (2.0 * math.pi) * bins))
        ranges[index] = min(ranges[index], distance_m)
    return tuple(ranges)
