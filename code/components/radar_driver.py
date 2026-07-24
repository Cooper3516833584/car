#!/usr/bin/env python3
"""D500/STL-19P radar acquisition, odometry and drone-frame mapping.

Coordinate convention (kept identical to the drone project):

* ``+X`` points forward and ``+Y`` points left.
* yaw is measured in degrees and is positive clockwise (a right turn).
* radar ranges are decoded in millimetres; poses and maps use centimetres.

The component never assumes that the car's local-map heading is equal to the
drone's global heading.  :class:`DroneGlobalAlignment` is calibrated from a
pair of corresponding poses and applies the required fixed SE(2) rotation and
translation to both poses and map points.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import math
import os
import select
import struct
import threading
import time
from typing import Callable, Final, Iterable, Sequence

try:
    import fcntl
    import termios
except ModuleNotFoundError:  # Protocol/coordinate tests also run on Windows.
    fcntl = None
    termios = None


DEFAULT_D500_PORT: Final[str] = "/dev/ttyS6"
DEFAULT_D500_BAUDRATE: Final[int] = 230400
D500_HEADER: Final[bytes] = b"\x54\x2C"
D500_FRAME_SIZE: Final[int] = 47
D500_POINT_COUNT: Final[int] = 12


class RadarDriverError(RuntimeError):
    """The UART, parser, or optional localization dependency failed."""


class GlobalCorrectionMode(Enum):
    """Where an accepted absolute wall correction is absorbed."""

    LEGACY_REWRITE_ODOMETRY = "legacy_rewrite_odometry"
    UPDATE_ALIGNMENT = "update_alignment"


def normalize_yaw_cw_deg(angle_deg: float) -> float:
    """Normalize a clockwise-positive angle to ``[-180, 180)``."""

    normalized = (float(angle_deg) + 180.0) % 360.0 - 180.0
    return 0.0 if abs(normalized) < 1e-12 else normalized


def rotate_cw(x: float, y: float, yaw_cw_deg: float) -> tuple[float, float]:
    """Rotate an XY vector using the drone clockwise-positive convention."""

    angle = math.radians(yaw_cw_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return cosine * x + sine * y, -sine * x + cosine * y


def calculate_d500_crc8(data: bytes) -> int:
    """Calculate the STL-19P CRC-8 (polynomial ``0x4D``, initial value 0)."""

    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


@dataclass(frozen=True, slots=True)
class Pose2D:
    """2-D pose in centimetres with clockwise-positive yaw in degrees."""

    x_cm: float = 0.0
    y_cm: float = 0.0
    yaw_cw_deg: float = 0.0


@dataclass(frozen=True, slots=True)
class RadarMount:
    """Radar origin and heading expressed in the car body frame.

    ``yaw_cw_deg=0`` means the radar's zero-degree ray points toward the car's
    front.  Positive values mean that the radar is mounted clockwise/right of
    the car's front.
    """

    x_forward_cm: float = 0.0
    y_left_cm: float = 0.0
    yaw_cw_deg: float = 0.0

    def sensor_to_body(self, point_cm: tuple[float, float]) -> tuple[float, float]:
        x_cm, y_cm = rotate_cw(*point_cm, self.yaw_cw_deg)
        return x_cm + self.x_forward_cm, y_cm + self.y_left_cm


@dataclass(frozen=True, slots=True)
class RadarPoint:
    angle_cw_deg: float
    distance_mm: int
    confidence: int

    def sensor_xy_cm(self) -> tuple[float, float]:
        distance_cm = self.distance_mm / 10.0
        angle = math.radians(self.angle_cw_deg)
        # Raw angles grow clockwise.  Cartesian +Y is left, hence the minus.
        return distance_cm * math.cos(angle), -distance_cm * math.sin(angle)


@dataclass(frozen=True, slots=True)
class RadarPacket:
    rotation_speed_deg_s: int
    start_angle_cw_deg: float
    stop_angle_cw_deg: float
    timestamp_ms: int
    points: tuple[RadarPoint, ...]


@dataclass(frozen=True, slots=True)
class RadarScan:
    """One assembled revolution in radar-sensor coordinates."""

    points: tuple[RadarPoint, ...]
    timestamp_ms: int
    rotation_speed_deg_s: int


@dataclass(slots=True)
class RadarParserStats:
    decoded_frames: int = 0
    crc_errors: int = 0
    discarded_bytes: int = 0


class D500PacketParser:
    """Incremental, self-resynchronizing D500 47-byte frame parser."""

    _PAYLOAD = struct.Struct("<HH" + "HB" * D500_POINT_COUNT + "HH")

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.stats = RadarParserStats()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> list[RadarPacket]:
        if data:
            self._buffer.extend(data)
        packets: list[RadarPacket] = []
        while True:
            header_index = self._buffer.find(D500_HEADER)
            if header_index < 0:
                keep = 1 if self._buffer[-1:] == D500_HEADER[:1] else 0
                discarded = len(self._buffer) - keep
                self.stats.discarded_bytes += discarded
                if keep:
                    del self._buffer[:-1]
                else:
                    self._buffer.clear()
                return packets
            if header_index:
                self.stats.discarded_bytes += header_index
                del self._buffer[:header_index]
            if len(self._buffer) < D500_FRAME_SIZE:
                return packets

            frame = bytes(self._buffer[:D500_FRAME_SIZE])
            if calculate_d500_crc8(frame[:-1]) != frame[-1]:
                self.stats.crc_errors += 1
                self.stats.discarded_bytes += 1
                del self._buffer[0]
                continue

            packets.append(self._decode(frame))
            self.stats.decoded_frames += 1
            del self._buffer[:D500_FRAME_SIZE]

    @classmethod
    def _decode(cls, frame: bytes) -> RadarPacket:
        values = cls._PAYLOAD.unpack_from(frame, len(D500_HEADER))
        speed, start_raw = values[:2]
        stop_raw, timestamp_ms = values[-2:]
        start_deg = start_raw / 100.0
        stop_deg = stop_raw / 100.0
        span_deg = (stop_deg - start_deg) % 360.0
        step_deg = span_deg / (D500_POINT_COUNT - 1)
        points = tuple(
            RadarPoint(
                angle_cw_deg=(start_deg + index * step_deg) % 360.0,
                distance_mm=values[2 + 2 * index],
                confidence=values[3 + 2 * index],
            )
            for index in range(D500_POINT_COUNT)
        )
        return RadarPacket(speed, start_deg, stop_deg, timestamp_ms, points)


class RadarScanAssembler:
    """Group consecutive packets into complete angular revolutions."""

    def __init__(
        self,
        *,
        min_distance_mm: int = 100,
        max_distance_mm: int = 6500,
        min_confidence: int = 30,
        min_points: int = 30,
    ) -> None:
        if not 0 <= min_distance_mm < max_distance_mm:
            raise ValueError("invalid radar distance limits")
        if not 0 <= min_confidence <= 255:
            raise ValueError("min_confidence must be in [0, 255]")
        self.min_distance_mm = min_distance_mm
        self.max_distance_mm = max_distance_mm
        self.min_confidence = min_confidence
        self.min_points = min_points
        self._points: list[RadarPoint] = []
        self._last_angle: float | None = None
        self._synchronized = False
        self._last_timestamp_ms = 0
        self._last_rotation_speed_deg_s = 0

    def reset(self) -> None:
        self._points.clear()
        self._last_angle = None
        self._synchronized = False
        self._last_timestamp_ms = 0
        self._last_rotation_speed_deg_s = 0

    def feed(self, packet: RadarPacket) -> list[RadarScan]:
        scans: list[RadarScan] = []
        for point in packet.points:
            if self._last_angle is not None and point.angle_cw_deg < self._last_angle - 180.0:
                # Data collection can start at any angle.  The first wrap only
                # synchronizes us to zero; it must not emit a partial scan.
                if self._synchronized and len(self._points) >= self.min_points:
                    scans.append(
                        RadarScan(
                            tuple(self._points),
                            self._last_timestamp_ms,
                            self._last_rotation_speed_deg_s,
                        )
                    )
                self._points = []
                self._synchronized = True
            self._last_angle = point.angle_cw_deg
            if (
                self.min_distance_mm <= point.distance_mm <= self.max_distance_mm
                and point.confidence >= self.min_confidence
            ):
                self._points.append(point)
        self._last_timestamp_ms = packet.timestamp_ms
        self._last_rotation_speed_deg_s = packet.rotation_speed_deg_s
        return scans


@dataclass(frozen=True, slots=True)
class DroneGlobalAlignment:
    """Fixed transform from the car local map to the drone global map."""

    x_offset_cm: float
    y_offset_cm: float
    yaw_offset_cw_deg: float

    @classmethod
    def from_reference(
        cls,
        car_local_pose: Pose2D,
        drone_global_pose: Pose2D,
    ) -> "DroneGlobalAlignment":
        """Calibrate from the same physical pose expressed in both frames."""

        yaw_offset = normalize_yaw_cw_deg(
            drone_global_pose.yaw_cw_deg - car_local_pose.yaw_cw_deg
        )
        rotated_x, rotated_y = rotate_cw(
            car_local_pose.x_cm, car_local_pose.y_cm, yaw_offset
        )
        return cls(
            drone_global_pose.x_cm - rotated_x,
            drone_global_pose.y_cm - rotated_y,
            yaw_offset,
        )

    def point_to_global(self, point_cm: tuple[float, float]) -> tuple[float, float]:
        x_cm, y_cm = rotate_cw(*point_cm, self.yaw_offset_cw_deg)
        return x_cm + self.x_offset_cm, y_cm + self.y_offset_cm

    def pose_to_global(self, pose: Pose2D) -> Pose2D:
        x_cm, y_cm = self.point_to_global((pose.x_cm, pose.y_cm))
        return Pose2D(
            x_cm,
            y_cm,
            normalize_yaw_cw_deg(pose.yaw_cw_deg + self.yaw_offset_cw_deg),
        )

    def point_to_local(self, point_cm: tuple[float, float]) -> tuple[float, float]:
        shifted_x = point_cm[0] - self.x_offset_cm
        shifted_y = point_cm[1] - self.y_offset_cm
        return rotate_cw(shifted_x, shifted_y, -self.yaw_offset_cw_deg)

    def pose_to_local(self, pose: Pose2D) -> Pose2D:
        x_cm, y_cm = self.point_to_local((pose.x_cm, pose.y_cm))
        return Pose2D(
            x_cm,
            y_cm,
            normalize_yaw_cw_deg(pose.yaw_cw_deg - self.yaw_offset_cw_deg),
        )


@dataclass(frozen=True, slots=True)
class RectangularWallReference:
    """Known back/right walls in a rectangular field coordinate frame.

    The wall frame follows the drone convention: +X forward from the back wall,
    +Y left from the right wall, and clockwise-positive yaw.  Usually the back
    wall is ``x=0`` and the right wall is ``y=0``.  ``wall_to_global`` permits
    the rectangular field to be translated or rotated relative to drone global.
    """

    wall_to_global: DroneGlobalAlignment
    back_wall_x_cm: float = 0.0
    right_wall_y_cm: float = 0.0
    front_wall_x_cm: float | None = None
    left_wall_y_cm: float | None = None


@dataclass(frozen=True, slots=True)
class WallLineConfig:
    association_gate_cm: float = 45.0
    inlier_gate_cm: float = 7.5
    min_points_per_wall: int = 12
    min_line_span_cm: float = 50.0
    max_line_rms_cm: float = 4.0
    max_axis_error_deg: float = 18.0
    max_wall_angle_disagreement_deg: float = 8.0

    def __post_init__(self) -> None:
        if min(
            self.association_gate_cm,
            self.inlier_gate_cm,
            self.min_line_span_cm,
            self.max_line_rms_cm,
            self.max_axis_error_deg,
            self.max_wall_angle_disagreement_deg,
        ) <= 0:
            raise ValueError("wall-line thresholds must be positive")
        if self.min_points_per_wall < 2:
            raise ValueError("min_points_per_wall must be at least two")


@dataclass(frozen=True, slots=True)
class WallPoseObservation:
    """Absolute pose observation in the rectangular wall frame."""

    x_cm: float | None
    y_cm: float | None
    yaw_cw_deg: float | None
    back_wall_points: int = 0
    right_wall_points: int = 0
    back_wall_rms_cm: float | None = None
    right_wall_rms_cm: float | None = None

    @property
    def observed_axes(self) -> int:
        return int(self.x_cm is not None) + int(self.y_cm is not None)


@dataclass(frozen=True, slots=True)
class _WallFit:
    points: int
    line_angle_ccw_deg: float
    coordinate_cm: float
    rms_cm: float


def _line_angle_delta_deg(angle_deg: float, expected_axis_deg: float) -> float:
    """Smallest undirected-line angle delta, in ``[-90, 90)``."""

    return (angle_deg - expected_axis_deg + 90.0) % 180.0 - 90.0


class WallLineLocalizer:
    """Absolute rectangular-wall observer inspired by ``radar_resolve_rt_pose``.

    Unlike the former image/Hough helper, association is guided by the current
    ICP prediction.  Robust PCA fits the known back (constant X) and right
    (constant Y) walls and returns an observation in the wall frame.  It does
    not update odometry by itself.
    """

    def __init__(
        self,
        reference: RectangularWallReference,
        *,
        mount: RadarMount = RadarMount(),
        config: WallLineConfig = WallLineConfig(),
    ) -> None:
        self.reference = reference
        self.mount = mount
        self.config = config

    def observe(
        self,
        scan: RadarScan,
        predicted_global_pose: Pose2D,
    ) -> WallPoseObservation:
        try:
            import numpy as np
        except ModuleNotFoundError as exc:
            raise RadarDriverError("wall-line localization requires numpy") from exc

        predicted_wall_pose = self.reference.wall_to_global.pose_to_local(
            predicted_global_pose
        )
        body_points = np.asarray(scan_points_in_body(scan, self.mount), dtype=float)
        if len(body_points) < self.config.min_points_per_wall:
            return WallPoseObservation(None, None, None)

        provisional = self._points_in_wall(body_points, predicted_wall_pose, np)
        x_fit = self._fit_nearest_wall(
            provisional,
            axis=0,
            wall_coordinates=self._x_wall_coordinates(),
            expected_line_angle_deg=90.0,
            np=np,
        )
        y_fit = self._fit_nearest_wall(
            provisional,
            axis=1,
            wall_coordinates=self._y_wall_coordinates(),
            expected_line_angle_deg=0.0,
            np=np,
        )
        back_fit = None if x_fit is None else x_fit[0]
        right_fit = None if y_fit is None else y_fit[0]

        angle_deltas: list[tuple[float, int]] = []
        if back_fit is not None:
            angle_deltas.append(
                (_line_angle_delta_deg(back_fit.line_angle_ccw_deg, 90.0), back_fit.points)
            )
        if right_fit is not None:
            angle_deltas.append(
                (_line_angle_delta_deg(right_fit.line_angle_ccw_deg, 0.0), right_fit.points)
            )
        if not angle_deltas:
            return WallPoseObservation(None, None, None)
        if len(angle_deltas) == 2 and abs(
            _line_angle_delta_deg(angle_deltas[0][0], angle_deltas[1][0])
        ) > self.config.max_wall_angle_disagreement_deg:
            return WallPoseObservation(None, None, None)

        total_weight = sum(weight for _, weight in angle_deltas)
        yaw_delta_ccw_deg = sum(delta * weight for delta, weight in angle_deltas) / total_weight
        observed_yaw_cw_deg = normalize_yaw_cw_deg(
            predicted_wall_pose.yaw_cw_deg + yaw_delta_ccw_deg
        )

        # Refit after correcting yaw so wall coordinates are not biased by the
        # prediction's angular drift.
        yaw_corrected_pose = Pose2D(
            predicted_wall_pose.x_cm,
            predicted_wall_pose.y_cm,
            observed_yaw_cw_deg,
        )
        corrected_points = self._points_in_wall(body_points, yaw_corrected_pose, np)
        x_fit = self._fit_nearest_wall(
            corrected_points,
            axis=0,
            wall_coordinates=self._x_wall_coordinates(),
            expected_line_angle_deg=90.0,
            np=np,
        )
        y_fit = self._fit_nearest_wall(
            corrected_points,
            axis=1,
            wall_coordinates=self._y_wall_coordinates(),
            expected_line_angle_deg=0.0,
            np=np,
        )
        back_fit = None if x_fit is None else x_fit[0]
        right_fit = None if y_fit is None else y_fit[0]

        observed_x = None
        if x_fit is not None:
            back_fit, x_wall_coordinate = x_fit
            observed_x = (
                predicted_wall_pose.x_cm
                + x_wall_coordinate
                - back_fit.coordinate_cm
            )
        observed_y = None
        if y_fit is not None:
            right_fit, y_wall_coordinate = y_fit
            observed_y = (
                predicted_wall_pose.y_cm
                + y_wall_coordinate
                - right_fit.coordinate_cm
            )
        return WallPoseObservation(
            observed_x,
            observed_y,
            observed_yaw_cw_deg,
            0 if back_fit is None else back_fit.points,
            0 if right_fit is None else right_fit.points,
            None if back_fit is None else back_fit.rms_cm,
            None if right_fit is None else right_fit.rms_cm,
        )

    def _x_wall_coordinates(self) -> tuple[float, ...]:
        coordinates = [self.reference.back_wall_x_cm]
        if self.reference.front_wall_x_cm is not None:
            coordinates.append(self.reference.front_wall_x_cm)
        return tuple(coordinates)

    def _y_wall_coordinates(self) -> tuple[float, ...]:
        coordinates = [self.reference.right_wall_y_cm]
        if self.reference.left_wall_y_cm is not None:
            coordinates.append(self.reference.left_wall_y_cm)
        return tuple(coordinates)

    def _fit_nearest_wall(
        self,
        points,
        *,
        axis: int,
        wall_coordinates: Sequence[float],
        expected_line_angle_deg: float,
        np,
    ) -> tuple[_WallFit, float] | None:
        fits: list[tuple[_WallFit, float]] = []
        for wall_coordinate in wall_coordinates:
            fit = self._fit_wall(
                points,
                axis=axis,
                wall_coordinate=wall_coordinate,
                expected_line_angle_deg=expected_line_angle_deg,
                np=np,
            )
            if fit is not None:
                fits.append((fit, wall_coordinate))
        if not fits:
            return None
        return min(
            fits,
            key=lambda item: (
                abs(item[0].coordinate_cm - item[1]),
                item[0].rms_cm,
                -item[0].points,
            ),
        )

    @staticmethod
    def _points_in_wall(body_points, pose: Pose2D, np):
        angle = math.radians(pose.yaw_cw_deg)
        rotation = np.array(
            [[math.cos(angle), math.sin(angle)], [-math.sin(angle), math.cos(angle)]]
        )
        return body_points @ rotation.T + np.array([pose.x_cm, pose.y_cm])

    def _fit_wall(
        self,
        points,
        *,
        axis: int,
        wall_coordinate: float,
        expected_line_angle_deg: float,
        np,
    ) -> _WallFit | None:
        distances = np.abs(points[:, axis] - wall_coordinate)
        candidates = points[distances <= self.config.association_gate_cm]
        if len(candidates) < self.config.min_points_per_wall:
            return None

        median_coordinate = float(np.median(candidates[:, axis]))
        inliers = candidates[
            np.abs(candidates[:, axis] - median_coordinate) <= self.config.inlier_gate_cm
        ]
        if len(inliers) < self.config.min_points_per_wall:
            return None
        centered = inliers - inliers.mean(axis=0)
        _, _, vt_matrix = np.linalg.svd(centered, full_matrices=False)
        direction = vt_matrix[0]
        line_angle = math.degrees(math.atan2(float(direction[1]), float(direction[0])))
        axis_error = abs(_line_angle_delta_deg(line_angle, expected_line_angle_deg))
        if axis_error > self.config.max_axis_error_deg:
            return None
        projections = centered @ direction
        if float(projections.max() - projections.min()) < self.config.min_line_span_cm:
            return None
        normal = np.array([-direction[1], direction[0]])
        perpendicular = centered @ normal
        rms = float(np.sqrt(np.mean(perpendicular**2)))
        if rms > self.config.max_line_rms_cm:
            return None
        return _WallFit(
            len(inliers),
            line_angle,
            float(np.median(inliers[:, axis])),
            rms,
        )


@dataclass(frozen=True, slots=True)
class RectangleCalibrationConfig:
    line_inlier_gate_cm: float = 4.0
    min_points_per_line: int = 18
    min_line_span_cm: float = 60.0
    max_line_rms_cm: float = 3.5
    max_ransac_trials: int = 1800
    max_lines: int = 8
    max_axis_error_deg: float = 15.0
    min_field_size_cm: float = 100.0
    max_sample_points: int = 600

    def __post_init__(self) -> None:
        if min(
            self.line_inlier_gate_cm,
            self.min_line_span_cm,
            self.max_line_rms_cm,
            self.max_axis_error_deg,
            self.min_field_size_cm,
        ) <= 0:
            raise ValueError("rectangle calibration thresholds must be positive")
        if self.min_points_per_line < 2 or self.max_ransac_trials <= 0:
            raise ValueError("invalid rectangle calibration sample limits")
        if self.max_lines < 4 or self.max_sample_points < self.min_points_per_line:
            raise ValueError("rectangle calibration needs at least four lines")


@dataclass(frozen=True, slots=True)
class RectangleFieldCalibration:
    """Fitted rectangular field, alignment, wall reference and safe boundary."""

    local_to_global: DroneGlobalAlignment
    wall_reference: RectangularWallReference
    initial_global_pose: Pose2D
    min_x_cm: float
    max_x_cm: float
    min_y_cm: float
    max_y_cm: float
    selected_edge_ccw_from_car_deg: float
    fitted_lines: int
    field_corners_cm: tuple[tuple[float, float], ...] = ()

    @property
    def field_polygon_cm(self) -> tuple[tuple[float, float], ...]:
        """Return the fitted field boundary in this calibration's global frame."""

        if self.field_corners_cm:
            return self.field_corners_cm
        return (
            (self.min_x_cm, self.min_y_cm),
            (self.max_x_cm, self.min_y_cm),
            (self.max_x_cm, self.max_y_cm),
            (self.min_x_cm, self.max_y_cm),
        )

    def contains_point(self, x_cm: float, y_cm: float, *, tolerance_cm: float = 1e-6) -> bool:
        """Return whether a point is inside/on the convex fitted rectangle."""

        polygon = self.field_polygon_cm
        if len(polygon) < 3:
            return False
        expected_sign = 0
        for index, first in enumerate(polygon):
            second = polygon[(index + 1) % len(polygon)]
            cross = (
                (second[0] - first[0]) * (y_cm - first[1])
                - (second[1] - first[1]) * (x_cm - first[0])
            )
            if abs(cross) <= tolerance_cm:
                continue
            sign = 1 if cross > 0 else -1
            if expected_sign == 0:
                expected_sign = sign
            elif sign != expected_sign:
                return False
        return True


@dataclass(frozen=True, slots=True)
class _CalibrationLine:
    center_x_cm: float
    center_y_cm: float
    direction_x: float
    direction_y: float
    points: int
    span_cm: float
    rms_cm: float

    @property
    def angle_ccw_deg(self) -> float:
        return math.degrees(math.atan2(self.direction_y, self.direction_x))


class RectangleFieldCalibrator:
    """Fit the four field edges while the car is stationary at startup."""

    def __init__(
        self,
        *,
        mount: RadarMount = RadarMount(),
        config: RectangleCalibrationConfig = RectangleCalibrationConfig(),
    ) -> None:
        self.mount = mount
        self.config = config

    def calibrate(self, scans: Sequence[RadarScan]) -> RectangleFieldCalibration:
        try:
            import numpy as np
        except ModuleNotFoundError as exc:
            raise RadarDriverError("rectangle field calibration requires numpy") from exc
        if not scans:
            raise RadarDriverError("rectangle calibration requires radar scans")
        combined = [point for scan in scans for point in scan_points_in_body(scan, self.mount)]
        if len(combined) < self.config.min_points_per_line * 4:
            raise RadarDriverError("not enough points to fit four field edges")
        points = np.asarray(combined, dtype=float)
        if len(points) > self.config.max_sample_points:
            indices = np.linspace(0, len(points) - 1, self.config.max_sample_points, dtype=int)
            points = points[indices]

        lines = self._extract_lines(points, np)
        if len(lines) < 4:
            raise RadarDriverError("fewer than four reliable rectangle edges were found")
        edge_angle = self._dominant_axis_angle(lines)
        local_to_global = DroneGlobalAlignment(0.0, 0.0, edge_angle)

        x_constants: list[float] = []
        y_constants: list[float] = []
        for line in lines:
            center_x, center_y = rotate_cw(
                line.center_x_cm, line.center_y_cm, edge_angle
            )
            direction_x, direction_y = rotate_cw(
                line.direction_x, line.direction_y, edge_angle
            )
            angle = math.degrees(math.atan2(direction_y, direction_x))
            if abs(_line_angle_delta_deg(angle, 0.0)) <= self.config.max_axis_error_deg:
                y_constants.append(center_y)
            elif abs(_line_angle_delta_deg(angle, 90.0)) <= self.config.max_axis_error_deg:
                x_constants.append(center_x)

        x_constants = self._merge_constants(x_constants)
        y_constants = self._merge_constants(y_constants)
        if len(x_constants) < 2 or len(y_constants) < 2:
            raise RadarDriverError("both pairs of rectangle edges were not found")
        min_x, max_x = min(x_constants), max(x_constants)
        min_y, max_y = min(y_constants), max(y_constants)
        if (
            max_x - min_x < self.config.min_field_size_cm
            or max_y - min_y < self.config.min_field_size_cm
        ):
            raise RadarDriverError("fitted rectangle is smaller than configured field minimum")

        initial_pose = local_to_global.pose_to_global(Pose2D())
        reference = RectangularWallReference(
            DroneGlobalAlignment(0.0, 0.0, 0.0),
            back_wall_x_cm=min_x,
            right_wall_y_cm=min_y,
            front_wall_x_cm=max_x,
            left_wall_y_cm=max_y,
        )
        return RectangleFieldCalibration(
            local_to_global,
            reference,
            initial_pose,
            min_x,
            max_x,
            min_y,
            max_y,
            edge_angle,
            len(lines),
        )

    def _extract_lines(self, points, np) -> list[_CalibrationLine]:
        remaining = points.copy()
        output: list[_CalibrationLine] = []
        rng = np.random.default_rng(0)
        for _ in range(self.config.max_lines):
            if len(remaining) < self.config.min_points_per_line:
                break
            best_mask = None
            best_score = -1.0
            for _ in range(self.config.max_ransac_trials):
                first, second = rng.integers(0, len(remaining), size=2)
                if first == second:
                    continue
                vector = remaining[second] - remaining[first]
                length = float(np.linalg.norm(vector))
                if length < self.config.min_line_span_cm * 0.5:
                    continue
                direction = vector / length
                normal = np.array([-direction[1], direction[0]])
                distances = np.abs((remaining - remaining[first]) @ normal)
                mask = distances <= self.config.line_inlier_gate_cm
                count = int(mask.sum())
                if count < self.config.min_points_per_line:
                    continue
                projected = remaining[mask] @ direction
                span = float(projected.max() - projected.min())
                if span < self.config.min_line_span_cm:
                    continue
                score = count + min(span, 500.0) * 0.02
                if score > best_score:
                    best_score, best_mask = score, mask
            if best_mask is None:
                break

            inliers = remaining[best_mask]
            center = inliers.mean(axis=0)
            centered = inliers - center
            _, _, vt_matrix = np.linalg.svd(centered, full_matrices=False)
            direction = vt_matrix[0]
            projections = centered @ direction
            span = float(projections.max() - projections.min())
            normal = np.array([-direction[1], direction[0]])
            rms = float(np.sqrt(np.mean((centered @ normal) ** 2)))
            if span >= self.config.min_line_span_cm and rms <= self.config.max_line_rms_cm:
                output.append(
                    _CalibrationLine(
                        float(center[0]),
                        float(center[1]),
                        float(direction[0]),
                        float(direction[1]),
                        len(inliers),
                        span,
                        rms,
                    )
                )
            remaining = remaining[~best_mask]
        return output

    @staticmethod
    def _dominant_axis_angle(lines: Sequence[_CalibrationLine]) -> float:
        # Rectangle directions are periodic every 90 degrees.  Mapping to
        # [-45,45) selects the edge direction closest to the initial car front.
        sine_sum = 0.0
        cosine_sum = 0.0
        for line in lines:
            candidate = (line.angle_ccw_deg + 45.0) % 90.0 - 45.0
            phase = math.radians(candidate * 4.0)
            weight = line.points * line.span_cm
            sine_sum += math.sin(phase) * weight
            cosine_sum += math.cos(phase) * weight
        return math.degrees(math.atan2(sine_sum, cosine_sum)) / 4.0

    def _merge_constants(self, values: Sequence[float]) -> list[float]:
        merged: list[list[float]] = []
        for value in sorted(values):
            if (
                not merged
                or abs(value - sum(merged[-1]) / len(merged[-1]))
                > self.config.line_inlier_gate_cm * 2
            ):
                merged.append([value])
            else:
                merged[-1].append(value)
        return [sum(group) / len(group) for group in merged]


@dataclass(frozen=True, slots=True)
class WallFusionConfig:
    update_every_scans: int = 1
    position_gain: float = 0.20
    yaw_gain: float = 0.15
    max_position_residual_cm: float = 60.0
    max_yaw_residual_deg: float = 20.0
    max_position_correction_cm: float = 2.0
    max_yaw_correction_deg: float = 0.5
    consistency_samples: int = 3
    max_position_spread_cm: float = 3.0
    max_yaw_spread_deg: float = 1.5

    def __post_init__(self) -> None:
        if self.update_every_scans <= 0 or self.consistency_samples <= 0:
            raise ValueError("wall update and consistency sample counts must be positive")
        if not 0.0 < self.position_gain <= 1.0 or not 0.0 < self.yaw_gain <= 1.0:
            raise ValueError("wall fusion gains must be in (0, 1]")
        if min(
            self.max_position_residual_cm,
            self.max_yaw_residual_deg,
            self.max_position_correction_cm,
            self.max_yaw_correction_deg,
            self.max_position_spread_cm,
            self.max_yaw_spread_deg,
        ) <= 0:
            raise ValueError("wall fusion residual gates must be positive")


@dataclass(frozen=True, slots=True)
class WallFusionResult:
    attempted: bool
    accepted: bool
    observation: WallPoseObservation | None
    fused_global_pose: Pose2D
    reason: str | None = None
    correction_x_cm: float = 0.0
    correction_y_cm: float = 0.0
    correction_yaw_deg: float = 0.0
    residual_x_cm: float = 0.0
    residual_y_cm: float = 0.0
    residual_yaw_deg: float = 0.0


def fuse_wall_observation(
    predicted_global_pose: Pose2D,
    observation: WallPoseObservation,
    reference: RectangularWallReference,
    config: WallFusionConfig = WallFusionConfig(),
) -> WallFusionResult:
    """Gate and blend one absolute wall observation with an ICP prediction."""

    predicted_wall = reference.wall_to_global.pose_to_local(predicted_global_pose)
    if observation.observed_axes == 0 or observation.yaw_cw_deg is None:
        return WallFusionResult(True, False, observation, predicted_global_pose, "no valid wall axes")

    x_residual = 0.0
    y_residual = 0.0
    if observation.x_cm is not None:
        x_residual = observation.x_cm - predicted_wall.x_cm
        if abs(x_residual) > config.max_position_residual_cm:
            return WallFusionResult(True, False, observation, predicted_global_pose, "wall X residual gate")
    if observation.y_cm is not None:
        y_residual = observation.y_cm - predicted_wall.y_cm
        if abs(y_residual) > config.max_position_residual_cm:
            return WallFusionResult(True, False, observation, predicted_global_pose, "wall Y residual gate")
    correction_x = x_residual * config.position_gain
    correction_y = y_residual * config.position_gain
    correction_norm = math.hypot(correction_x, correction_y)
    if correction_norm > config.max_position_correction_cm:
        scale = config.max_position_correction_cm / correction_norm
        correction_x *= scale
        correction_y *= scale
    yaw_residual = normalize_yaw_cw_deg(
        observation.yaw_cw_deg - predicted_wall.yaw_cw_deg
    )
    if abs(yaw_residual) > config.max_yaw_residual_deg:
        return WallFusionResult(True, False, observation, predicted_global_pose, "wall yaw residual gate")
    yaw_correction = yaw_residual * config.yaw_gain
    yaw_correction = max(
        -config.max_yaw_correction_deg,
        min(config.max_yaw_correction_deg, yaw_correction),
    )
    fused_wall = Pose2D(
        predicted_wall.x_cm + correction_x,
        predicted_wall.y_cm + correction_y,
        normalize_yaw_cw_deg(predicted_wall.yaw_cw_deg + yaw_correction),
    )
    return WallFusionResult(
        True,
        True,
        observation,
        reference.wall_to_global.pose_to_global(fused_wall),
        correction_x_cm=correction_x,
        correction_y_cm=correction_y,
        correction_yaw_deg=yaw_correction,
        residual_x_cm=x_residual,
        residual_y_cm=y_residual,
        residual_yaw_deg=yaw_residual,
    )


def scan_points_in_body(
    scan: RadarScan,
    mount: RadarMount,
) -> list[tuple[float, float]]:
    return [mount.sensor_to_body(point.sensor_xy_cm()) for point in scan.points]


def body_points_to_local(
    points_cm: Iterable[tuple[float, float]],
    car_local_pose: Pose2D,
) -> list[tuple[float, float]]:
    output: list[tuple[float, float]] = []
    for point in points_cm:
        x_cm, y_cm = rotate_cw(*point, car_local_pose.yaw_cw_deg)
        output.append((x_cm + car_local_pose.x_cm, y_cm + car_local_pose.y_cm))
    return output


def scan_points_in_drone_global(
    scan: RadarScan,
    car_local_pose: Pose2D,
    mount: RadarMount,
    alignment: DroneGlobalAlignment,
) -> list[tuple[float, float]]:
    local_points = body_points_to_local(scan_points_in_body(scan, mount), car_local_pose)
    return [alignment.point_to_global(point) for point in local_points]


@dataclass(frozen=True, slots=True)
class ICPResult:
    transform_current_to_reference: Pose2D
    matched_points: int
    mean_error_cm: float
    iterations: int


class ICPScanMatcher:
    """SVD point-to-point ICP, following the drone radar localization method."""

    def __init__(
        self,
        *,
        max_correspondence_cm: float = 35.0,
        min_correspondences: int = 20,
        max_iterations: int = 20,
        tolerance_cm: float = 0.02,
        max_points: int = 360,
    ) -> None:
        self.max_correspondence_cm = max_correspondence_cm
        self.min_correspondences = min_correspondences
        self.max_iterations = max_iterations
        self.tolerance_cm = tolerance_cm
        self.max_points = max_points

    def match(
        self,
        reference_points_cm: Sequence[tuple[float, float]],
        current_points_cm: Sequence[tuple[float, float]],
    ) -> ICPResult:
        try:
            import numpy as np
        except ModuleNotFoundError as exc:
            raise RadarDriverError("ICP localization requires numpy") from exc

        reference = self._sample(np.asarray(reference_points_cm, dtype=float), np)
        current = self._sample(np.asarray(current_points_cm, dtype=float), np)
        if len(reference) < self.min_correspondences or len(current) < self.min_correspondences:
            raise RadarDriverError("not enough radar points for ICP")

        rotation = np.eye(2)
        translation = np.zeros(2)
        previous_error = math.inf
        matched = 0
        mean_error = math.inf
        iteration = 0
        max_distance_sq = self.max_correspondence_cm**2

        for iteration in range(1, self.max_iterations + 1):
            transformed = current @ rotation.T + translation
            distances_sq = ((transformed[:, None, :] - reference[None, :, :]) ** 2).sum(axis=2)
            nearest_indices = distances_sq.argmin(axis=1)
            nearest_distances_sq = distances_sq[np.arange(len(current)), nearest_indices]
            mask = nearest_distances_sq <= max_distance_sq
            matched = int(mask.sum())
            if matched < self.min_correspondences:
                raise RadarDriverError("too few ICP correspondences inside distance gate")

            source = transformed[mask]
            target = reference[nearest_indices[mask]]
            source_center = source.mean(axis=0)
            target_center = target.mean(axis=0)
            covariance = (source - source_center).T @ (target - target_center)
            u_matrix, _, vt_matrix = np.linalg.svd(covariance)
            delta_rotation = vt_matrix.T @ u_matrix.T
            if np.linalg.det(delta_rotation) < 0:
                vt_matrix[-1, :] *= -1
                delta_rotation = vt_matrix.T @ u_matrix.T
            delta_translation = target_center - delta_rotation @ source_center
            rotation = delta_rotation @ rotation
            translation = delta_rotation @ translation + delta_translation

            aligned = source @ delta_rotation.T + delta_translation
            mean_error = float(np.linalg.norm(aligned - target, axis=1).mean())
            if abs(previous_error - mean_error) <= self.tolerance_cm:
                break
            previous_error = mean_error

        yaw_cw_deg = normalize_yaw_cw_deg(
            math.degrees(math.atan2(float(rotation[0, 1]), float(rotation[0, 0])))
        )
        return ICPResult(
            Pose2D(float(translation[0]), float(translation[1]), yaw_cw_deg),
            matched,
            mean_error,
            iteration,
        )

    def _sample(self, points, np):
        if points.ndim != 2 or points.shape[1:] != (2,):
            raise ValueError("ICP points must have shape (N, 2)")
        if len(points) <= self.max_points:
            return points
        indices = np.linspace(0, len(points) - 1, self.max_points, dtype=int)
        return points[indices]


@dataclass(frozen=True, slots=True)
class RadarOdometryUpdate:
    pose: Pose2D
    accepted: bool
    initialized: bool
    icp: ICPResult | None = None
    rejection_reason: str | None = None


class RadarOdometry:
    """Incremental scan-to-scan radar odometry in the car local frame."""

    def __init__(
        self,
        *,
        mount: RadarMount = RadarMount(),
        matcher: ICPScanMatcher | None = None,
        max_step_cm: float = 15.0,
        max_step_yaw_deg: float = 15.0,
        max_mean_error_cm: float = 10.0,
        min_step_cm: float = 2.0,
        min_step_yaw_deg: float = 1.0,
        max_lateral_innovation_cm: float = 5.0,
    ) -> None:
        if min(
            max_step_cm,
            max_step_yaw_deg,
            max_mean_error_cm,
            min_step_cm,
            min_step_yaw_deg,
            max_lateral_innovation_cm,
        ) <= 0:
            raise ValueError("radar odometry gates must be positive")
        if min_step_cm >= max_step_cm or min_step_yaw_deg >= max_step_yaw_deg:
            raise ValueError("radar odometry minimum gates must be below maximum gates")
        self.mount = mount
        self.matcher = matcher or ICPScanMatcher()
        self.max_step_cm = max_step_cm
        self.max_step_yaw_deg = max_step_yaw_deg
        self.max_mean_error_cm = max_mean_error_cm
        self.min_step_cm = min_step_cm
        self.min_step_yaw_deg = min_step_yaw_deg
        self.max_lateral_innovation_cm = max_lateral_innovation_cm
        self.pose = Pose2D()
        self._reference: list[tuple[float, float]] | None = None

    def reset(self, pose: Pose2D = Pose2D()) -> None:
        self.pose = pose
        self._reference = None

    def update(self, scan: RadarScan) -> RadarOdometryUpdate:
        current = scan_points_in_body(scan, self.mount)
        if self._reference is None:
            self._reference = current
            return RadarOdometryUpdate(self.pose, True, True)
        try:
            result = self.matcher.match(self._reference, current)
        except (RadarDriverError, ValueError) as exc:
            return RadarOdometryUpdate(self.pose, False, True, rejection_reason=str(exc))

        delta = result.transform_current_to_reference
        if math.hypot(delta.x_cm, delta.y_cm) > self.max_step_cm:
            return RadarOdometryUpdate(self.pose, False, True, result, "translation gate")
        if result.mean_error_cm > self.max_mean_error_cm:
            return RadarOdometryUpdate(self.pose, False, True, result, "error gate")
        if (
            math.hypot(delta.x_cm, delta.y_cm) < self.min_step_cm
            and abs(delta.yaw_cw_deg) < self.min_step_yaw_deg
        ):
            # Preserve the keyframe so real low-speed motion accumulates while
            # stationary sub-centimetre ICP jitter cannot walk the pose.
            return RadarOdometryUpdate(self.pose, True, True, result)
        if abs(delta.yaw_cw_deg) > self.max_step_yaw_deg:
            return RadarOdometryUpdate(self.pose, False, True, result, "yaw gate")
        expected_lateral_cm = -delta.x_cm * math.tan(
            math.radians(delta.yaw_cw_deg) / 2.0
        )
        if abs(delta.y_cm - expected_lateral_cm) > self.max_lateral_innovation_cm:
            return RadarOdometryUpdate(
                self.pose,
                False,
                True,
                result,
                "Ackermann lateral gate",
            )

        delta_x, delta_y = rotate_cw(delta.x_cm, delta.y_cm, self.pose.yaw_cw_deg)
        self.pose = Pose2D(
            self.pose.x_cm + delta_x,
            self.pose.y_cm + delta_y,
            normalize_yaw_cw_deg(self.pose.yaw_cw_deg + delta.yaw_cw_deg),
        )
        self._reference = current
        return RadarOdometryUpdate(self.pose, True, True, result)


@dataclass(frozen=True, slots=True)
class MapCell:
    x_cm: float
    y_cm: float
    hits: int


class DroneGlobalPointMap:
    """Sparse hit-count map whose cells are always in drone global XY."""

    def __init__(self, resolution_cm: float = 5.0) -> None:
        if resolution_cm <= 0:
            raise ValueError("resolution_cm must be positive")
        self.resolution_cm = float(resolution_cm)
        self._hits: dict[tuple[int, int], int] = {}
        self._lock = threading.Lock()

    def clear(self) -> None:
        with self._lock:
            self._hits.clear()

    def add_points(self, global_points_cm: Iterable[tuple[float, float]]) -> None:
        with self._lock:
            for x_cm, y_cm in global_points_cm:
                key = (round(x_cm / self.resolution_cm), round(y_cm / self.resolution_cm))
                self._hits[key] = self._hits.get(key, 0) + 1

    def cells(self, *, min_hits: int = 1) -> list[MapCell]:
        with self._lock:
            return [
                MapCell(ix * self.resolution_cm, iy * self.resolution_cm, hits)
                for (ix, iy), hits in sorted(self._hits.items())
                if hits >= min_hits
            ]


@dataclass(frozen=True, slots=True)
class RadarLocalizationUpdate:
    scan: RadarScan
    odometry: RadarOdometryUpdate
    global_pose: Pose2D | None
    global_points_cm: tuple[tuple[float, float], ...] = field(default_factory=tuple)
    wall_fusion: WallFusionResult | None = None


def _safe_callback(callback: Callable | None, *args) -> None:
    if callback is not None:
        callback(*args)


class D500SerialDriver:
    """Read-only threaded UART driver with reconnect and packet callbacks."""

    def __init__(
        self,
        *,
        on_packet: Callable[[RadarPacket], None],
        port: str = DEFAULT_D500_PORT,
        reconnect_seconds: float = 1.0,
        on_connected: Callable[[], None] | None = None,
        on_disconnected: Callable[[BaseException | None], None] | None = None,
    ) -> None:
        self.on_packet = on_packet
        self.port = port
        self.reconnect_seconds = reconnect_seconds
        self.on_connected = on_connected
        self.on_disconnected = on_disconnected
        self.parser = D500PacketParser()
        self._fd: int | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._connected_event.is_set()

    def wait_connected(self, timeout: float | None = None) -> bool:
        return self._connected_event.wait(timeout)

    def start(self) -> "D500SerialDriver":
        if self._thread and self._thread.is_alive():
            return self
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="d500-uart", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop_event.set()
        self._close_fd(None)
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)

    def __enter__(self) -> "D500SerialDriver":
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._open()
                while not self._stop_event.is_set():
                    fd = self._fd
                    if fd is None:
                        break
                    readable, _, _ = select.select([fd], [], [], 0.2)
                    if not readable:
                        continue
                    chunk = os.read(fd, 4096)
                    if not chunk:
                        raise RadarDriverError("D500 UART returned EOF")
                    for packet in self.parser.feed(chunk):
                        _safe_callback(self.on_packet, packet)
            except BaseException as exc:
                if isinstance(exc, (KeyboardInterrupt, SystemExit)):
                    raise
                self._close_fd(exc)
                self._stop_event.wait(self.reconnect_seconds)
        self._close_fd(None)

    def _open(self) -> None:
        if termios is None or fcntl is None:
            raise RadarDriverError("D500 UART driver requires Linux termios")
        fd = os.open(self.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            attrs[4] = termios.B230400
            attrs[5] = termios.B230400
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIFLUSH)
            with self._lock:
                self._fd = fd
            self.parser.reset()
            self._connected_event.set()
            _safe_callback(self.on_connected)
        except BaseException:
            os.close(fd)
            raise

    def _close_fd(self, error: BaseException | None) -> None:
        with self._lock:
            fd, self._fd = self._fd, None
        was_connected = self._connected_event.is_set()
        self._connected_event.clear()
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
        if was_connected:
            _safe_callback(self.on_disconnected, error)


class D500RadarComponent:
    """UART -> scans -> ICP -> optional wall correction -> global map."""

    def __init__(
        self,
        *,
        port: str = DEFAULT_D500_PORT,
        mount: RadarMount = RadarMount(),
        alignment: DroneGlobalAlignment | None = None,
        on_update: Callable[[RadarLocalizationUpdate], None] | None = None,
        on_connected: Callable[[], None] | None = None,
        on_disconnected: Callable[[BaseException | None], None] | None = None,
        assembler: RadarScanAssembler | None = None,
        odometry: RadarOdometry | None = None,
        global_map: DroneGlobalPointMap | None = None,
        wall_localizer: WallLineLocalizer | None = None,
        wall_fusion_config: WallFusionConfig = WallFusionConfig(),
        global_correction_mode: GlobalCorrectionMode = GlobalCorrectionMode.LEGACY_REWRITE_ODOMETRY,
    ) -> None:
        if not isinstance(global_correction_mode, GlobalCorrectionMode):
            raise TypeError("global_correction_mode must be a GlobalCorrectionMode")
        self.mount = mount
        self._alignment = alignment
        self.on_update = on_update
        self.assembler = assembler or RadarScanAssembler()
        self.odometry = odometry or RadarOdometry(mount=mount)
        self.global_map = global_map or DroneGlobalPointMap()
        self.wall_localizer = wall_localizer
        self.wall_fusion_config = wall_fusion_config
        self.global_correction_mode = global_correction_mode
        self._state_lock = threading.RLock()
        self._wall_scan_count = 0
        self._wall_observation_history: deque[WallPoseObservation] = deque(
            maxlen=wall_fusion_config.consistency_samples
        )
        self.serial = D500SerialDriver(
            port=port,
            on_packet=self.process_packet,
            on_connected=on_connected,
            on_disconnected=on_disconnected,
        )

    @property
    def alignment(self) -> DroneGlobalAlignment | None:
        """Return a stable snapshot of the map-to-odom alignment."""

        return self.get_alignment()

    @alignment.setter
    def alignment(self, alignment: DroneGlobalAlignment | None) -> None:
        self.set_alignment(alignment)

    def set_alignment(self, alignment: DroneGlobalAlignment | None) -> None:
        """Safely replace the global alignment used by the processing thread."""

        if alignment is not None and not isinstance(alignment, DroneGlobalAlignment):
            raise TypeError("alignment must be a DroneGlobalAlignment or None")
        with self._state_lock:
            self._alignment = alignment

    def get_alignment(self) -> DroneGlobalAlignment | None:
        with self._state_lock:
            return self._alignment

    def set_global_reference(
        self,
        car_local_pose: Pose2D,
        drone_global_pose: Pose2D,
    ) -> DroneGlobalAlignment:
        alignment = DroneGlobalAlignment.from_reference(car_local_pose, drone_global_pose)
        self.set_alignment(alignment)
        return alignment

    def enable_wall_fusion(
        self,
        reference: RectangularWallReference,
        *,
        line_config: WallLineConfig = WallLineConfig(),
        fusion_config: WallFusionConfig = WallFusionConfig(),
    ) -> WallLineLocalizer:
        """Enable periodic absolute correction from known back/right walls."""

        with self._state_lock:
            self.wall_localizer = WallLineLocalizer(
                reference,
                mount=self.mount,
                config=line_config,
            )
            self.wall_fusion_config = fusion_config
            self._wall_scan_count = 0
            self._wall_observation_history = deque(
                maxlen=fusion_config.consistency_samples
            )
            return self.wall_localizer

    def disable_wall_fusion(self) -> None:
        with self._state_lock:
            self.wall_localizer = None
            self._wall_scan_count = 0
            self._wall_observation_history.clear()

    def _wall_observation_is_consistent(
        self,
        observation: WallPoseObservation,
        config: WallFusionConfig,
    ) -> bool:
        """Require repeated absolute observations before correcting odometry.

        Rectangular rooms contain several parallel walls.  A single erroneous
        association must therefore never move the vehicle pose.  Once the
        absolute observations agree, large *valid* residuals are recovered by
        the per-scan correction clamps in :func:`fuse_wall_observation`.
        """

        if observation.observed_axes == 0 or observation.yaw_cw_deg is None:
            self._wall_observation_history.clear()
            return False
        signature = (observation.x_cm is not None, observation.y_cm is not None)
        if self._wall_observation_history:
            previous = self._wall_observation_history[-1]
            previous_signature = (previous.x_cm is not None, previous.y_cm is not None)
            if previous_signature != signature:
                self._wall_observation_history.clear()
        self._wall_observation_history.append(observation)
        if len(self._wall_observation_history) < config.consistency_samples:
            return False

        history = tuple(self._wall_observation_history)
        for attribute in ("x_cm", "y_cm"):
            values = [
                float(value)
                for value in (getattr(item, attribute) for item in history)
                if value is not None
            ]
            if values and max(values) - min(values) > config.max_position_spread_cm:
                return False
        reference_yaw = float(history[0].yaw_cw_deg)
        yaw_deltas = [
            normalize_yaw_cw_deg(float(item.yaw_cw_deg) - reference_yaw)
            for item in history
        ]
        return max(yaw_deltas) - min(yaw_deltas) <= config.max_yaw_spread_deg

    def start(self) -> "D500RadarComponent":
        self.serial.start()
        return self

    def close(self) -> None:
        self.serial.close()

    def __enter__(self) -> "D500RadarComponent":
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def process_packet(self, packet: RadarPacket) -> list[RadarLocalizationUpdate]:
        """Process decoded input; public so recorded packets can be replayed."""

        updates: list[RadarLocalizationUpdate] = []
        for scan in self.assembler.feed(packet):
            odometry_update = self.odometry.update(scan)
            global_pose: Pose2D | None = None
            global_points: tuple[tuple[float, float], ...] = ()
            wall_fusion: WallFusionResult | None = None
            with self._state_lock:
                alignment = self._alignment
                wall_localizer = self.wall_localizer
                wall_fusion_config = self.wall_fusion_config
            if alignment is not None:
                global_pose = alignment.pose_to_global(odometry_update.pose)
                if odometry_update.accepted and wall_localizer is not None:
                    with self._state_lock:
                        self._wall_scan_count += 1
                        try_wall_fusion = (
                            self._wall_scan_count >= wall_fusion_config.update_every_scans
                        )
                        if try_wall_fusion:
                            self._wall_scan_count = 0
                    if try_wall_fusion:
                        try:
                            observation = wall_localizer.observe(scan, global_pose)
                            with self._state_lock:
                                consistent = self._wall_observation_is_consistent(
                                    observation,
                                    wall_fusion_config,
                                )
                                consensus_count = len(self._wall_observation_history)
                            if consistent:
                                wall_fusion = fuse_wall_observation(
                                    global_pose,
                                    observation,
                                    wall_localizer.reference,
                                    wall_fusion_config,
                                )
                            else:
                                wall_fusion = WallFusionResult(
                                    True,
                                    False,
                                    observation,
                                    global_pose,
                                    "wall observation consensus "
                                    f"{consensus_count}/{wall_fusion_config.consistency_samples}",
                                )
                        except (RadarDriverError, ValueError) as exc:
                            wall_fusion = WallFusionResult(
                                True,
                                False,
                                None,
                                global_pose,
                                str(exc),
                            )
                        if wall_fusion.accepted:
                            global_pose = wall_fusion.fused_global_pose
                            if self.global_correction_mode is GlobalCorrectionMode.LEGACY_REWRITE_ODOMETRY:
                                corrected_local_pose = alignment.pose_to_local(global_pose)
                                self.odometry.pose = corrected_local_pose
                                odometry_update = RadarOdometryUpdate(
                                    corrected_local_pose,
                                    True,
                                    odometry_update.initialized,
                                    odometry_update.icp,
                                    odometry_update.rejection_reason,
                                )
                            else:
                                # In ROS, map->odom carries the discontinuous
                                # wall correction and odom->base_link remains
                                # a continuous ICP trajectory for Nav2.
                                alignment = DroneGlobalAlignment.from_reference(
                                    odometry_update.pose,
                                    global_pose,
                                )
                                self.set_alignment(alignment)
                if odometry_update.accepted:
                    global_points = tuple(
                        scan_points_in_drone_global(
                            scan, odometry_update.pose, self.mount, alignment
                        )
                    )
                    self.global_map.add_points(global_points)
            update = RadarLocalizationUpdate(
                scan,
                odometry_update,
                global_pose,
                global_points,
                wall_fusion,
            )
            updates.append(update)
            _safe_callback(self.on_update, update)
        return updates


# Concise alias for callers that treat this as the project's radar driver.
RadarDriver = D500RadarComponent
