"""Fixed competition-track geometry and Pure Pursuit following."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import IntEnum
import logging
import math
import threading
import time
from typing import Callable, Final

from .ackermann_drive import AckermannDrive
from .navigation import (
    NavigationGoal,
    NavigationPath,
    NavigationPose,
    PathPoint,
    PurePursuitConfig,
    PurePursuitController,
    radar_yaw_to_navigation_heading,
)
from .radar_driver import RadarLocalizationUpdate
from .rear_motor import MotorDirection


LOG = logging.getLogger(__name__)


A_FIELD_CM: Final[tuple[float, float]] = (150.0, 200.0)
B_FIELD_CM: Final[tuple[float, float]] = (150.0, 350.0)
C_FIELD_CM: Final[tuple[float, float]] = (300.0, 350.0)
D_FIELD_CM: Final[tuple[float, float]] = (300.0, 200.0)
TRACK_RADIUS_CM: Final[float] = 75.0

S_A_CM: Final[float] = 0.0
S_B_CM: Final[float] = 150.0
S_C_CM: Final[float] = S_B_CM + math.pi * TRACK_RADIUS_CM
S_D_CM: Final[float] = 300.0 + math.pi * TRACK_RADIUS_CM
S_FINISH_CM: Final[float] = 300.0 + 2.0 * math.pi * TRACK_RADIUS_CM

TRACK_SAMPLE_SPACING_CM: Final[float] = 2.5
WRAP_EXTENSION_CM: Final[float] = 100.0
FINISH_APPROACH_DISTANCE_CM: Final[float] = 40.0
FINISH_APPROACH_SPEED_CM_S: Final[float] = 8.0
FINISH_POSITION_TOLERANCE_CM: Final[float] = 4.0
FINISH_CROSS_TRACK_TOLERANCE_CM: Final[float] = 3.0
FINISH_HEADING_TOLERANCE_DEG: Final[float] = 6.0
FINISH_MAX_OVERSHOOT_CM: Final[float] = 15.0


class TrackSegment(IntEnum):
    AB = 0
    BC = 1
    CD = 2
    DA = 3


@dataclass(frozen=True, slots=True)
class CompetitionTrackSpeedProfile:
    """Forward speed for each labelled portion of the D-task black loop."""

    ab_cm_s: float
    bc_cm_s: float
    cd_cm_s: float
    da_cm_s: float

    def __post_init__(self) -> None:
        for name, value in (
            ("ab_cm_s", self.ab_cm_s),
            ("bc_cm_s", self.bc_cm_s),
            ("cd_cm_s", self.cd_cm_s),
            ("da_cm_s", self.da_cm_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")

    @classmethod
    def uniform(cls, speed_cm_s: float) -> "CompetitionTrackSpeedProfile":
        return cls(speed_cm_s, speed_cm_s, speed_cm_s, speed_cm_s)

    @property
    def max_speed_cm_s(self) -> float:
        return max(self.ab_cm_s, self.bc_cm_s, self.cd_cm_s, self.da_cm_s)

    def for_segment(self, segment: TrackSegment) -> float:
        return (
            self.ab_cm_s,
            self.bc_cm_s,
            self.cd_cm_s,
            self.da_cm_s,
        )[int(segment)]


TRACK_PURSUIT_CONFIG: Final[PurePursuitConfig] = PurePursuitConfig(
    cruise_speed_mm_s=500.0,
    max_speed_mm_s=600.0,
    approach_speed_mm_s=80.0,
    reverse_speed_mm_s=80.0,
    min_lookahead_cm=24.0,
    max_lookahead_cm=36.0,
    slowdown_distance_cm=1.0,
    max_path_deviation_cm=35.0,
    cross_track_gain=0.25,
    heading_gain=0.55,
    feedback_softening_speed_cm_s=5.0,
    cross_track_slowdown_cm=20.0,
    heading_slowdown_deg=35.0,
    minimum_tracking_speed_scale=1.0,
    nearest_search_ahead_points=60,
)


@dataclass(frozen=True, slots=True)
class CompetitionTrackPoint:
    x_cm: float
    y_cm: float
    heading_deg: float
    progress_cm: float
    segment: TrackSegment


@dataclass(frozen=True, slots=True)
class TrackFollowerState:
    running: bool
    completed: bool
    segment: TrackSegment
    progress_cm: float
    target_speed_cm_s: float
    commanded_speed_cm_s: float
    steering_angle_rad: float
    cross_track_error_cm: float
    heading_error_deg: float


def line_reference_to_rear_axle(
    line_x_cm: float,
    line_y_cm: float,
    heading_math_rad: float,
    *,
    offset_cm: float = 0.0,
) -> tuple[float, float]:
    """Translate a measured black-line reference point to the rear axle."""

    return (
        line_x_cm - offset_cm * math.cos(heading_math_rad),
        line_y_cm - offset_cm * math.sin(heading_math_rad),
    )


@dataclass(frozen=True, slots=True)
class FieldTransform:
    """Convert startup-relative Navigation coordinates to competition field."""

    start_rear_field_cm: tuple[float, float]
    start_field_heading_deg: float = 0.0

    @classmethod
    def from_a_reference(
        cls, *, offset_cm: float = 0.0
    ) -> "FieldTransform":
        rear = line_reference_to_rear_axle(
            *A_FIELD_CM,
            math.pi / 2.0,
            offset_cm=offset_cm,
        )
        return cls(rear, 0.0)

    def field_to_navigation(
        self, x_cm: float, y_cm: float, field_heading_deg: float
    ) -> NavigationPose:
        dx = float(x_cm) - self.start_rear_field_cm[0]
        dy = float(y_cm) - self.start_rear_field_cm[1]
        start_math = math.radians(90.0 - self.start_field_heading_deg)
        forward_x, forward_y = math.cos(start_math), math.sin(start_math)
        left_x, left_y = -forward_y, forward_x
        return NavigationPose(
            dx * forward_x + dy * forward_y,
            dx * left_x + dy * left_y,
            (self.start_field_heading_deg - field_heading_deg) % 360.0,
        )

    def navigation_to_field(
        self, pose: NavigationPose
    ) -> tuple[float, float, float]:
        start_math = math.radians(90.0 - self.start_field_heading_deg)
        forward_x, forward_y = math.cos(start_math), math.sin(start_math)
        left_x, left_y = -forward_y, forward_x
        return (
            self.start_rear_field_cm[0]
            + pose.x_cm * forward_x
            + pose.y_cm * left_x,
            self.start_rear_field_cm[1]
            + pose.x_cm * forward_y
            + pose.y_cm * left_y,
            (self.start_field_heading_deg - pose.heading_deg) % 360.0,
        )

class CompetitionTrack:
    def __init__(
        self,
        points: tuple[CompetitionTrackPoint, ...],
        path: NavigationPath,
        field_points_cm: tuple[tuple[float, float], ...],
        segment_start_indices: tuple[int, int, int, int],
        wrap_start_index: int,
        finish_progress_cm: float,
    ) -> None:
        self._points = points
        self._path = path
        self.field_points_cm = field_points_cm
        self.segment_start_indices = segment_start_indices
        self.wrap_start_index = wrap_start_index
        self._finish_progress_cm = finish_progress_cm
        self._finish_index = next(
            index
            for index, point in enumerate(points)
            if point.progress_cm >= finish_progress_cm
        )

    @classmethod
    def build(
        cls,
        *,
        reference_offset_cm: float,
        sample_spacing_cm: float = TRACK_SAMPLE_SPACING_CM,
        wrap_extension_cm: float = WRAP_EXTENSION_CM,
        finish_extension_cm: float = 0.0,
    ) -> "CompetitionTrack":
        if sample_spacing_cm <= 0.0 or wrap_extension_cm <= 0.0:
            raise ValueError("sample spacing and wrap extension must be positive")
        if (
            not math.isfinite(finish_extension_cm)
            or not 0.0 <= finish_extension_cm <= wrap_extension_cm
        ):
            raise ValueError(
                "finish extension must be finite and within the wrap extension"
            )
        transform = FieldTransform.from_a_reference(
            offset_cm=reference_offset_cm
        )
        raw_points: list[CompetitionTrackPoint] = []
        field_points: list[tuple[float, float]] = []
        starts: list[int] = []

        def append(
            line_x: float,
            line_y: float,
            field_heading: float,
            s_cm: float,
            segment: TrackSegment,
        ) -> None:
            heading_math = math.radians(90.0 - field_heading)
            field_x, field_y = line_reference_to_rear_axle(
                line_x,
                line_y,
                heading_math,
                offset_cm=reference_offset_cm,
            )
            pose = transform.field_to_navigation(
                field_x, field_y, field_heading
            )
            if field_points and math.hypot(
                field_x - field_points[-1][0],
                field_y - field_points[-1][1],
            ) <= 1e-9:
                return
            field_points.append((field_x, field_y))
            raw_points.append(
                CompetitionTrackPoint(
                    pose.x_cm,
                    pose.y_cm,
                    pose.heading_deg,
                    s_cm,
                    segment,
                )
            )

        straight_samples = max(1, math.ceil(S_B_CM / sample_spacing_cm))
        starts.append(0)
        for index in range(straight_samples + 1):
            ratio = index / straight_samples
            append(
                150.0,
                200.0 + 150.0 * ratio,
                0.0,
                150.0 * ratio,
                TrackSegment.AB,
            )

        arc_samples = max(
            2, math.ceil(math.pi * TRACK_RADIUS_CM / sample_spacing_cm)
        )
        starts.append(len(raw_points) - 1)
        raw_points[-1] = replace(raw_points[-1], segment=TrackSegment.BC)
        for index in range(1, arc_samples + 1):
            ratio = index / arc_samples
            angle = math.pi - math.pi * ratio
            append(
                225.0 + TRACK_RADIUS_CM * math.cos(angle),
                350.0 + TRACK_RADIUS_CM * math.sin(angle),
                180.0 * ratio,
                S_B_CM + math.pi * TRACK_RADIUS_CM * ratio,
                TrackSegment.BC,
            )

        starts.append(len(raw_points) - 1)
        raw_points[-1] = replace(raw_points[-1], segment=TrackSegment.CD)
        for index in range(1, straight_samples + 1):
            ratio = index / straight_samples
            append(
                300.0,
                350.0 - 150.0 * ratio,
                180.0,
                S_C_CM + 150.0 * ratio,
                TrackSegment.CD,
            )

        starts.append(len(raw_points) - 1)
        raw_points[-1] = replace(raw_points[-1], segment=TrackSegment.DA)
        for index in range(1, arc_samples + 1):
            ratio = index / arc_samples
            angle = -math.pi * ratio
            append(
                225.0 + TRACK_RADIUS_CM * math.cos(angle),
                200.0 + TRACK_RADIUS_CM * math.sin(angle),
                180.0 + 180.0 * ratio,
                S_D_CM + math.pi * TRACK_RADIUS_CM * ratio,
                TrackSegment.DA,
            )

        wrap_start_index = len(raw_points) - 1
        # A is also the first point of the repeated AB segment.
        raw_points[wrap_start_index] = replace(
            raw_points[wrap_start_index],
            segment=TrackSegment.AB,
        )
        wrap_samples = max(1, math.ceil(wrap_extension_cm / sample_spacing_cm))
        wrap_distances = {
            wrap_extension_cm * index / wrap_samples
            for index in range(1, wrap_samples + 1)
        }
        if finish_extension_cm > 0.0:
            # Preserve the exact requested stop point even when it is not an
            # integer multiple of the regular path-sampling interval.
            wrap_distances.add(finish_extension_cm)
        for distance in sorted(wrap_distances):
            append(
                A_FIELD_CM[0],
                A_FIELD_CM[1] + distance,
                0.0,
                S_FINISH_CM + distance,
                TrackSegment.AB,
            )

        path_points = tuple(
            PathPoint(
                x_cm=point.x_cm,
                y_cm=point.y_cm,
                heading_deg=point.heading_deg,
                direction=MotorDirection.FORWARD,
            )
            for point in raw_points
        )
        goal = NavigationGoal(
            path_points[-1].x_cm,
            path_points[-1].y_cm,
            final_heading_deg=path_points[-1].heading_deg,
        )
        return cls(
            tuple(raw_points),
            NavigationPath(path_points, goal),
            tuple(field_points),
            tuple(starts),
            wrap_start_index,
            S_FINISH_CM + finish_extension_cm,
        )

    @property
    def points(self) -> tuple[CompetitionTrackPoint, ...]:
        return self._points

    @property
    def path(self) -> NavigationPath:
        return self._path

    @property
    def finish_progress_cm(self) -> float:
        return self._finish_progress_cm

    @property
    def finish_point(self) -> CompetitionTrackPoint:
        return self._points[self._finish_index]

    def point_at_index(self, index: int) -> CompetitionTrackPoint:
        return self._points[index]

    def segment_at_progress(self, progress_cm: float) -> TrackSegment:
        if progress_cm < S_B_CM:
            return TrackSegment.AB
        if progress_cm < S_C_CM:
            return TrackSegment.BC
        if progress_cm < S_D_CM:
            return TrackSegment.CD
        if progress_cm < S_FINISH_CM:
            return TrackSegment.DA
        return TrackSegment.AB

    def segment_for_index(self, index: int) -> TrackSegment:
        return self._points[index].segment


def build_competition_track(
    *,
    sample_spacing_cm: float = TRACK_SAMPLE_SPACING_CM,
    wrap_extension_cm: float = WRAP_EXTENSION_CM,
    reference_offset_cm: float = 0.0,
    finish_extension_cm: float = 0.0,
    transform: FieldTransform | None = None,
) -> CompetitionTrack:
    if transform is not None:
        expected = FieldTransform.from_a_reference(
            offset_cm=reference_offset_cm
        )
        if transform != expected:
            raise ValueError(
                "custom transform must match the configured A reference"
            )
    return CompetitionTrack.build(
        reference_offset_cm=reference_offset_cm,
        sample_spacing_cm=sample_spacing_cm,
        wrap_extension_cm=wrap_extension_cm,
        finish_extension_cm=finish_extension_cm,
    )


class CompetitionTrackFollower:
    """Use existing Pure Pursuit only when a new accepted radar pose arrives."""

    def __init__(
        self,
        *,
        drive: AckermannDrive,
        track: CompetitionTrack,
        speed_cm_s: float | None = None,
        speed_profile: CompetitionTrackSpeedProfile | None = None,
        controller: PurePursuitController | None = None,
        on_state_changed: Callable[[TrackFollowerState], None] | None = None,
    ) -> None:
        if (speed_cm_s is None) == (speed_profile is None):
            raise ValueError("provide exactly one of speed_cm_s or speed_profile")
        self.drive = drive
        self.speed_profile = (
            CompetitionTrackSpeedProfile.uniform(float(speed_cm_s))
            if speed_profile is None
            else speed_profile
        )
        self._track = track
        self._controller = controller or PurePursuitController(
            config=TRACK_PURSUIT_CONFIG
        )
        self._on_state_changed = on_state_changed
        self._running = False
        self._completed = False
        self._terminal_tolerance_met = False
        self._terminal_hard_stop_triggered = False
        self._progress_index = 0
        self._lock = threading.Lock()
        self._state = TrackFollowerState(
            False,
            False,
            TrackSegment.AB,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    @property
    def state(self) -> TrackFollowerState:
        with self._lock:
            return self._state

    @property
    def track(self) -> CompetitionTrack:
        return self._track

    @property
    def progress_index(self) -> int:
        with self._lock:
            return self._progress_index

    @property
    def progress_cm(self) -> float:
        with self._lock:
            return self._state.progress_cm

    @property
    def terminal_tolerance_met(self) -> bool:
        with self._lock:
            return self._terminal_tolerance_met

    @property
    def terminal_hard_stop_triggered(self) -> bool:
        with self._lock:
            return self._terminal_hard_stop_triggered

    def start_mission(self) -> None:
        with self._lock:
            if self._running or self._completed:
                return
            self._running = True
            self._terminal_tolerance_met = False
            self._terminal_hard_stop_triggered = False
            self._progress_index = 0
            self._state = replace(
                self._state,
                running=True,
                segment=TrackSegment.AB,
                progress_cm=0.0,
                target_speed_cm_s=self.speed_profile.ab_cm_s,
            )
            state = self._state
        self._notify(state)

    def update_from_radar(
        self,
        update: RadarLocalizationUpdate,
    ) -> TrackFollowerState:
        if update.global_pose is None or not update.odometry.accepted:
            return self.state
        pose = NavigationPose(
            x_cm=update.global_pose.x_cm,
            y_cm=update.global_pose.y_cm,
            heading_deg=radar_yaw_to_navigation_heading(
                update.global_pose.yaw_cw_deg
            ),
            timestamp_s=time.monotonic(),
        )
        with self._lock:
            if not self._running or self._completed:
                return self._state
            try:
                command = self._controller.compute(
                    pose,
                    self._track.path,
                    min_path_index=self._progress_index,
                )
                previous_segment = self._state.segment
                self._progress_index = max(
                    self._progress_index,
                    command.nearest_path_index,
                )
                point = self._track.point_at_index(self._progress_index)
                progress_cm = max(
                    self._state.progress_cm,
                    point.progress_cm,
                )
                finish_goal = self._track.finish_point
                finish_distance_cm = math.hypot(
                    pose.x_cm - finish_goal.x_cm,
                    pose.y_cm - finish_goal.y_cm,
                )
                finish_index_reached = (
                    progress_cm >= self._track.finish_progress_cm
                    and self._progress_index >= self._track.wrap_start_index
                )
                terminal_tolerance_met = (
                    finish_index_reached
                    and finish_distance_cm
                    <= FINISH_POSITION_TOLERANCE_CM
                    and abs(command.cross_track_error_cm)
                    <= FINISH_CROSS_TRACK_TOLERANCE_CM
                    and abs(command.heading_error_deg)
                    <= FINISH_HEADING_TOLERANCE_DEG
                )
                terminal_hard_stop = (
                    progress_cm
                    >= self._track.finish_progress_cm
                    + FINISH_MAX_OVERSHOOT_CM
                )
                if terminal_tolerance_met or terminal_hard_stop:
                    self.drive.stop(center_steering=True)
                    self._running = False
                    self._completed = True
                    self._terminal_tolerance_met = terminal_tolerance_met
                    self._terminal_hard_stop_triggered = (
                        terminal_hard_stop
                        and not terminal_tolerance_met
                    )
                    self._state = TrackFollowerState(
                        False,
                        True,
                        point.segment,
                        progress_cm,
                        0.0,
                        0.0,
                        command.steering_angle_rad,
                        command.cross_track_error_cm,
                        command.heading_error_deg,
                    )
                    state = self._state
                    if terminal_tolerance_met:
                        LOG.info(
                            "competition track complete progress_cm=%.2f "
                            "path_index=%d finish_distance_cm=%.2f "
                            "cross_track_cm=%.2f heading_error_deg=%.2f",
                            progress_cm,
                            self._progress_index,
                            finish_distance_cm,
                            command.cross_track_error_cm,
                            command.heading_error_deg,
                        )
                    else:
                        LOG.warning(
                            "competition track terminal hard stop "
                            "progress_cm=%.2f path_index=%d "
                            "finish_distance_cm=%.2f cross_track_cm=%.2f "
                            "heading_error_deg=%.2f",
                            progress_cm,
                            self._progress_index,
                            finish_distance_cm,
                            command.cross_track_error_cm,
                            command.heading_error_deg,
                        )
                else:
                    cruise_speed_cm_s = self.speed_profile.for_segment(
                        point.segment
                    )
                    remaining_cm = max(
                        0.0,
                        self._track.finish_progress_cm - progress_cm,
                    )
                    if remaining_cm >= FINISH_APPROACH_DISTANCE_CM:
                        target_speed_cm_s = cruise_speed_cm_s
                    else:
                        approach_speed_cm_s = min(
                            cruise_speed_cm_s,
                            FINISH_APPROACH_SPEED_CM_S,
                        )
                        blend = (
                            remaining_cm / FINISH_APPROACH_DISTANCE_CM
                        )
                        target_speed_cm_s = approach_speed_cm_s + (
                            cruise_speed_cm_s - approach_speed_cm_s
                        ) * blend
                    plan = self.drive.set_motion(
                        target_speed_cm_s * 10.0,
                        command.steering_angle_rad,
                        direction=MotorDirection.FORWARD,
                        rear_differential_linked=True,
                    )
                    commanded_speed_cm_s = (
                        abs(plan.center_speed_mm_s) / 10.0
                    )
                    self._state = TrackFollowerState(
                        True,
                        False,
                        point.segment,
                        progress_cm,
                        target_speed_cm_s,
                        commanded_speed_cm_s,
                        command.steering_angle_rad,
                        command.cross_track_error_cm,
                        command.heading_error_deg,
                    )
                    state = self._state
                    if point.segment is not previous_segment:
                        LOG.info(
                            "track segment changed %s -> %s progress_cm=%.2f",
                            previous_segment.name,
                            point.segment.name,
                            progress_cm,
                        )
                    LOG.debug(
                        "track segment=%s progress_cm=%.2f "
                        "target_speed_cm_s=%.2f commanded_speed_cm_s=%.2f "
                        "steering_rad=%.4f cross_track_cm=%.2f "
                        "heading_error_deg=%.2f path_index=%d",
                        point.segment.name,
                        progress_cm,
                        target_speed_cm_s,
                        commanded_speed_cm_s,
                        command.steering_angle_rad,
                        command.cross_track_error_cm,
                        command.heading_error_deg,
                        self._progress_index,
                    )
            except BaseException:
                self.drive.stop(center_steering=True)
                self._running = False
                self._state = replace(
                    self._state,
                    running=False,
                    target_speed_cm_s=0.0,
                    commanded_speed_cm_s=0.0,
                )
                raise
        self._notify(state)
        return state

    def stop_mission(self) -> None:
        with self._lock:
            if self._running:
                self.drive.stop(center_steering=True)
            self._running = False
            self._state = replace(
                self._state,
                running=False,
                target_speed_cm_s=0.0,
                commanded_speed_cm_s=0.0,
            )
            state = self._state
        self._notify(state)

    def _notify(self, state: TrackFollowerState) -> None:
        if self._on_state_changed is not None:
            self._on_state_changed(state)
