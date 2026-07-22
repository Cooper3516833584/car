#!/usr/bin/env python3
"""Unified speed, yaw, steering-servo, and rear-wheel control."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import Final

from .rear_motor import (
    ChassisCommand,
    MotorDirection,
    RearMotorDriver,
    WheelSpeeds,
    wheel_speeds_to_chassis,
)
from .steering_servo import (
    FrontSteeringServo,
    SteeringCommand,
    YawDirection,
    make_steering_command,
    yaw_to_steering_command,
)


DEFAULT_WHEELBASE_MM: Final[float] = 142.5
DEFAULT_TRACK_WIDTH_MM: Final[float] = 117.1
DEFAULT_FIRMWARE_TRACK_WIDTH_MM: Final[float] = 164.0


@dataclass(frozen=True, slots=True)
class AckermannMotionPlan:
    """Fully validated front-steering and rear-motor targets."""

    center_speed_mm_s: float
    steering: SteeringCommand
    rear: ChassisCommand
    rear_differential_linked: bool
    turn_radius_mm: float | None


def plan_ackermann_motion(
    speed_mm_s: float,
    steering_angle_rad: float,
    *,
    direction: MotorDirection = MotorDirection.FORWARD,
    rear_differential_linked: bool = True,
    max_wheel_speed_mm_s: float = 300.0,
    wheelbase_mm: float = DEFAULT_WHEELBASE_MM,
    track_width_mm: float = DEFAULT_TRACK_WIDTH_MM,
    firmware_track_width_mm: float = DEFAULT_FIRMWARE_TRACK_WIDTH_MM,
    min_turn_radius_mm: float = 350.0,
) -> AckermannMotionPlan:
    """Validate and calculate one coordinated Ackermann motion command.

    ``speed_mm_s`` is the non-negative vehicle-centre speed.  In linked mode,
    rear-wheel differential speed is derived from the calibrated front-wheel
    steering angle.  With linkage disabled both rear wheels receive the same
    signed speed while the front servo still turns.
    """

    speed = float(speed_mm_s)
    wheelbase = float(wheelbase_mm)
    track = float(track_width_mm)
    firmware_track = float(firmware_track_width_mm)
    if not math.isfinite(speed) or speed < 0.0:
        raise ValueError("speed_mm_s must be finite and non-negative")
    if not isinstance(direction, MotorDirection):
        raise TypeError("direction must be a MotorDirection")
    if not math.isfinite(wheelbase) or wheelbase <= 0.0:
        raise ValueError("wheelbase_mm must be finite and positive")
    if not math.isfinite(track) or track <= 0.0:
        raise ValueError("track_width_mm must be finite and positive")
    if not math.isfinite(firmware_track) or firmware_track <= 0.0:
        raise ValueError("firmware_track_width_mm must be finite and positive")

    steering = make_steering_command(steering_angle_rad)
    signed_speed = speed * direction.value
    radius: float | None = None
    if rear_differential_linked and not math.isclose(
        steering.angle_rad, 0.0, abs_tol=1e-12
    ):
        # The calibrated steering angle corresponds to the physical car, so
        # radius and requested wheel differential use measured dimensions.
        radius = wheelbase / math.tan(steering.angle_rad) - track / 2.0
        if signed_speed != 0.0 and abs(radius) + 1e-9 < min_turn_radius_mm:
            raise ValueError(
                f"physical turn radius {abs(radius):.1f} mm is below the "
                f"configured vehicle minimum {min_turn_radius_mm:.1f} mm"
            )
        angular_rad_s = signed_speed / radius if signed_speed != 0.0 else 0.0
        half_delta = angular_rad_s * track / 2.0
        left = signed_speed - half_delta
        right = signed_speed + half_delta
    else:
        left = right = signed_speed

    rear = wheel_speeds_to_chassis(
        left,
        right,
        max_wheel_speed_mm_s=max_wheel_speed_mm_s,
        # C10B converts Vz back to rear-wheel differential with its compiled
        # 164 mm track.  This is deliberately distinct from the measured
        # physical wheel-centre track used above.
        track_width_mm=firmware_track,
        min_turn_radius_mm=min_turn_radius_mm,
    )
    return AckermannMotionPlan(
        center_speed_mm_s=signed_speed,
        steering=steering,
        rear=rear,
        rear_differential_linked=rear_differential_linked,
        turn_radius_mm=radius,
    )


class AckermannDrive:
    """Compose front steering and rear motors into one safe drive component.

    The rear-motor watchdog remains active, so motion commands must be refreshed
    before ``RearMotorDriver.command_timeout_s`` expires.
    """

    def __init__(
        self,
        rear_motors: RearMotorDriver | None = None,
        steering: FrontSteeringServo | None = None,
        *,
        wheelbase_mm: float = DEFAULT_WHEELBASE_MM,
        track_width_mm: float = DEFAULT_TRACK_WIDTH_MM,
        firmware_track_width_mm: float | None = None,
        max_wheel_speed_mm_s: float | None = None,
    ) -> None:
        if rear_motors is None:
            firmware_track = (
                DEFAULT_FIRMWARE_TRACK_WIDTH_MM
                if firmware_track_width_mm is None
                else float(firmware_track_width_mm)
            )
            rear_speed_limit = (
                300.0
                if max_wheel_speed_mm_s is None
                else float(max_wheel_speed_mm_s)
            )
            self.rear_motors = RearMotorDriver(
                track_width_mm=firmware_track,
                max_wheel_speed_mm_s=rear_speed_limit,
            )
        else:
            self.rear_motors = rear_motors
            firmware_track = float(rear_motors.track_width_mm)
            if firmware_track_width_mm is not None and not math.isclose(
                float(firmware_track_width_mm),
                firmware_track,
                abs_tol=1e-9,
            ):
                raise ValueError(
                    "firmware_track_width_mm does not match rear motor protocol geometry"
                )
            if max_wheel_speed_mm_s is not None and not math.isclose(
                float(max_wheel_speed_mm_s),
                rear_motors.max_wheel_speed_mm_s,
                abs_tol=1e-9,
            ):
                raise ValueError(
                    "max_wheel_speed_mm_s does not match supplied rear motor driver"
                )
        self.steering = steering or FrontSteeringServo()
        self.wheelbase_mm = float(wheelbase_mm)
        self.track_width_mm = float(track_width_mm)
        self.firmware_track_width_mm = firmware_track
        if not math.isfinite(self.wheelbase_mm) or self.wheelbase_mm <= 0.0:
            raise ValueError("wheelbase_mm must be finite and positive")
        if not math.isfinite(self.track_width_mm) or self.track_width_mm <= 0.0:
            raise ValueError("track_width_mm must be finite and positive")
        if (
            not math.isfinite(self.firmware_track_width_mm)
            or self.firmware_track_width_mm <= 0.0
        ):
            raise ValueError("firmware_track_width_mm must be finite and positive")
        self._lock = threading.Lock()
        self._started = False
        self._speed_mm_s = 0.0
        self._direction = MotorDirection.FORWARD
        self._rear_differential_linked = True

    @property
    def is_running(self) -> bool:
        return self._started and self.rear_motors.is_running and self.steering.is_running

    def start(self) -> "AckermannDrive":
        with self._lock:
            if self._started:
                raise RuntimeError("Ackermann drive is already running")
            self.steering.start()
            try:
                self.rear_motors.start()
            except BaseException:
                self.steering.close()
                raise
            self._started = True
        return self

    def __enter__(self) -> "AckermannDrive":
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def set_motion(
        self,
        speed_mm_s: float,
        steering_angle_rad: float,
        *,
        direction: MotorDirection = MotorDirection.FORWARD,
        rear_differential_linked: bool = True,
    ) -> AckermannMotionPlan:
        """Set vehicle speed and signed steering angle in one coordinated call."""

        with self._lock:
            self._require_started()
            plan = self._plan(
                speed_mm_s,
                steering_angle_rad,
                direction,
                rear_differential_linked,
            )
            self._apply_plan(plan)
            self._speed_mm_s = float(speed_mm_s)
            self._direction = direction
            self._rear_differential_linked = rear_differential_linked
            return plan

    def set_yaw(
        self,
        direction: YawDirection,
        magnitude_rad: float,
        *,
        rear_differential_linked: bool = True,
    ) -> AckermannMotionPlan:
        """Change yaw direction/magnitude and retain the latest centre speed."""

        steering = yaw_to_steering_command(direction, magnitude_rad)
        return self.set_motion(
            self._speed_mm_s,
            steering.angle_rad,
            direction=self._direction,
            rear_differential_linked=rear_differential_linked,
        )

    def set_steering(
        self,
        steering_angle_rad: float,
        *,
        rear_differential_linked: bool = True,
    ) -> AckermannMotionPlan:
        """Set a signed steering angle while retaining the latest centre speed."""

        return self.set_motion(
            self._speed_mm_s,
            steering_angle_rad,
            direction=self._direction,
            rear_differential_linked=rear_differential_linked,
        )

    def set_speed(
        self,
        speed_mm_s: float,
        direction: MotorDirection | None = None,
        *,
        rear_differential_linked: bool | None = None,
    ) -> AckermannMotionPlan:
        """Change speed while retaining and, by default, linking current steering."""

        chosen_direction = direction or self._direction
        chosen_linkage = (
            self._rear_differential_linked
            if rear_differential_linked is None
            else rear_differential_linked
        )
        return self.set_motion(
            speed_mm_s,
            self.steering.command.angle_rad,
            direction=chosen_direction,
            rear_differential_linked=chosen_linkage,
        )

    def center_steering(self, *, keep_speed: bool = True) -> AckermannMotionPlan:
        """Return front wheels to centre, optionally retaining vehicle speed."""

        return self.set_motion(
            self._speed_mm_s if keep_speed else 0.0,
            0.0,
            direction=self._direction,
            rear_differential_linked=True,
        )

    def stop(self, *, center_steering: bool = True) -> None:
        """Immediately stop both rear motors and optionally centre the front wheels."""

        with self._lock:
            self._require_started()
            self.rear_motors.stop()
            self._speed_mm_s = 0.0
            if center_steering:
                self.steering.center()

    def close(self) -> None:
        with self._lock:
            if not self._started:
                return
            try:
                self.rear_motors.close()
            finally:
                self.steering.close()
                self._started = False
                self._speed_mm_s = 0.0

    def _plan(
        self,
        speed_mm_s: float,
        steering_angle_rad: float,
        direction: MotorDirection,
        rear_differential_linked: bool,
    ) -> AckermannMotionPlan:
        return plan_ackermann_motion(
            speed_mm_s,
            steering_angle_rad,
            direction=direction,
            rear_differential_linked=rear_differential_linked,
            max_wheel_speed_mm_s=self.rear_motors.max_wheel_speed_mm_s,
            wheelbase_mm=self.wheelbase_mm,
            track_width_mm=self.track_width_mm,
            firmware_track_width_mm=self.firmware_track_width_mm,
            min_turn_radius_mm=self.rear_motors.min_turn_radius_mm,
        )

    def _apply_plan(self, plan: AckermannMotionPlan) -> None:
        # Planning has already validated both targets.  Steering is written
        # first; the rear target follows immediately and is refreshed at 20 Hz.
        self.steering.apply(plan.steering)
        requested: WheelSpeeds = plan.rear.requested
        try:
            self.rear_motors.set_wheels(
                requested.left_mm_s,
                requested.right_mm_s,
            )
        except BaseException:
            # If the rear channel fails, do not leave a steering-only motion
            # command active.  Best-effort stop and return to centre.
            try:
                self.rear_motors.stop()
            except BaseException:
                pass
            try:
                self.steering.center()
            except BaseException:
                pass
            raise

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError("Ackermann drive is not running")
