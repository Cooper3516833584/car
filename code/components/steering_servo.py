#!/usr/bin/env python3
"""Calibrated front-steering servo for the Ackermann car.

The steering *math* (cubic factory curve, direction inversion, PWM pulse
calculation) lives here and accepts a :class:`SteeringCalibration` object.
The calibration values, the PWM output and the mechanical limits belong to the
vehicle profile / HAL layer and are injected by the composition root; this
module no longer knows ROCK 5A pin names or the physical board layout.

``DEFAULT_STEERING_CALIBRATION`` mirrors the verified Cooper ROCK 5A +
WHEELTEC L150 profile so pure unit tests and legacy call sites keep working.
Production entries always build the calibration from the TOML profile.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Final, Protocol

from hal.pwm import PWMOutput


@dataclass(frozen=True, slots=True)
class SteeringCalibration:
    """All servo calibration data for one physical car."""

    direction_sign: float = -1.0
    logical_right_max_rad: float = -0.32
    logical_left_max_rad: float = 0.49
    calibration_min_rad: float = -0.49
    calibration_max_rad: float = 0.32
    pwm_min_us: int = 800
    pwm_max_us: int = 2200
    factory_center_us: int = 1501
    center_us: int = 1580
    curve_a3: float = -0.628
    curve_a2: float = 1.269
    curve_a1: float = -1.772
    curve_a0: float = 1.573
    curve_scale: float = 640.62

    def __post_init__(self) -> None:
        values = (
            self.direction_sign,
            self.logical_right_max_rad,
            self.logical_left_max_rad,
            self.calibration_min_rad,
            self.calibration_max_rad,
            self.curve_a3,
            self.curve_a2,
            self.curve_a1,
            self.curve_a0,
            self.curve_scale,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("steering calibration values must be finite")
        if self.direction_sign not in (-1.0, 1.0):
            raise ValueError("direction_sign must be +1 or -1")
        if self.logical_right_max_rad >= 0.0:
            raise ValueError("logical_right_max_rad must be negative")
        if self.logical_left_max_rad <= 0.0:
            raise ValueError("logical_left_max_rad must be positive")
        if not self.calibration_min_rad < self.calibration_max_rad:
            raise ValueError("calibration_min_rad must be below calibration_max_rad")
        if not self.pwm_min_us < self.center_us < self.pwm_max_us:
            raise ValueError("pwm must satisfy pwm_min_us < center_us < pwm_max_us")
        if self.factory_center_us <= 0 or self.curve_scale <= 0.0:
            raise ValueError("factory_center_us and curve_scale must be positive")


# Backward-compatible default = the verified current car profile.  The formal
# main entries inject the calibration built from the TOML configuration.
DEFAULT_STEERING_CALIBRATION: Final[SteeringCalibration] = SteeringCalibration()

# Legacy module-level aliases kept for existing tests and call sites.
STEERING_DIRECTION_SIGN: Final[float] = DEFAULT_STEERING_CALIBRATION.direction_sign
STEERING_RIGHT_MAX_RAD: Final[float] = (
    DEFAULT_STEERING_CALIBRATION.logical_right_max_rad
)
STEERING_LEFT_MAX_RAD: Final[float] = (
    DEFAULT_STEERING_CALIBRATION.logical_left_max_rad
)
CALIBRATION_MIN_RAD: Final[float] = (
    DEFAULT_STEERING_CALIBRATION.calibration_min_rad
)
CALIBRATION_MAX_RAD: Final[float] = (
    DEFAULT_STEERING_CALIBRATION.calibration_max_rad
)
FACTORY_CENTER_US: Final[int] = DEFAULT_STEERING_CALIBRATION.factory_center_us
STEERING_CENTER_US: Final[int] = DEFAULT_STEERING_CALIBRATION.center_us


class SteeringStateError(RuntimeError):
    """The steering PWM component is not started or cannot access PWM."""


class YawDirection(Enum):
    """Vehicle yaw convention: positive/left, negative/right."""

    LEFT = 1
    RIGHT = -1


@dataclass(frozen=True, slots=True)
class SteeringCommand:
    """One validated steering target and its calibrated PWM pulse width."""

    angle_rad: float
    pulse_us: int

    @property
    def yaw_direction(self) -> YawDirection | None:
        if self.angle_rad > 0.0:
            return YawDirection.LEFT
        if self.angle_rad < 0.0:
            return YawDirection.RIGHT
        return None


def _finite_angle(
    angle_rad: float,
    calibration: SteeringCalibration,
) -> float:
    angle = float(angle_rad)
    if not math.isfinite(angle):
        raise ValueError("angle_rad must be finite")
    if not calibration.logical_right_max_rad <= angle <= calibration.logical_left_max_rad:
        raise ValueError(
            "angle_rad must be in "
            f"[{calibration.logical_right_max_rad}, "
            f"{calibration.logical_left_max_rad}]"
        )
    return angle


def steering_angle_to_pulse_us(
    angle_rad: float,
    calibration: SteeringCalibration = DEFAULT_STEERING_CALIBRATION,
) -> int:
    """Apply the WHEELTEC L150 cubic steering-angle calibration.

    Vehicle steering remains positive/left and negative/right.  Real-car logs
    plus direct wheel observation confirmed that this servo/linkage is mounted
    opposite to the factory curve: logical left therefore evaluates the curve
    at a negative calibration angle and produces a pulse above centre.
    """

    angle = _finite_angle(angle_rad, calibration)
    calibration_angle = calibration.direction_sign * angle
    if not calibration.calibration_min_rad <= calibration_angle <= calibration.calibration_max_rad:
        raise ValueError("logical steering angle exceeds calibration travel")
    servo_angle = (
        calibration.curve_a3 * calibration_angle**3
        + calibration.curve_a2 * calibration_angle**2
        + calibration.curve_a1 * calibration_angle
        + calibration.curve_a0
    )
    factory_pulse = 1500.0 + (
        servo_angle - (calibration.curve_a0 - 0.001)
    ) * calibration.curve_scale
    pulse = factory_pulse + (calibration.center_us - calibration.factory_center_us)
    return round(
        max(
            calibration.pwm_min_us,
            min(calibration.pwm_max_us, pulse),
        )
    )


def make_steering_command(
    angle_rad: float,
    calibration: SteeringCalibration = DEFAULT_STEERING_CALIBRATION,
) -> SteeringCommand:
    angle = _finite_angle(angle_rad, calibration)
    return SteeringCommand(
        angle,
        steering_angle_to_pulse_us(angle, calibration),
    )


def yaw_to_steering_command(
    direction: YawDirection,
    magnitude_rad: float,
    calibration: SteeringCalibration = DEFAULT_STEERING_CALIBRATION,
) -> SteeringCommand:
    """Build a command from an explicit yaw direction and angle magnitude."""

    if not isinstance(direction, YawDirection):
        raise TypeError("direction must be a YawDirection")
    magnitude = float(magnitude_rad)
    if not math.isfinite(magnitude) or magnitude < 0.0:
        raise ValueError("magnitude_rad must be finite and non-negative")
    return make_steering_command(direction.value * magnitude, calibration)


class FrontSteeringServo:
    """PWM controller for the front steering servo.

    The PWM output (``PWMOutput``) is provided by the HAL layer and already
    carries the board-specific configuration.  Starting centres and enables
    the servo.  Closing returns to centre and deliberately leaves PWM enabled
    so the front wheels continue to hold the safe centre position.
    """

    def __init__(
        self,
        calibration: SteeringCalibration = DEFAULT_STEERING_CALIBRATION,
        pwm: PWMOutput | None = None,
    ) -> None:
        if not isinstance(calibration, SteeringCalibration):
            raise TypeError("calibration must be a SteeringCalibration")
        self.calibration = calibration
        self._pwm = pwm
        self._started_pwm: PWMOutput | None = None
        self._command = make_steering_command(0.0, calibration)

    @property
    def command(self) -> SteeringCommand:
        return self._command

    @property
    def is_running(self) -> bool:
        return self._started_pwm is not None

    def start(self) -> "FrontSteeringServo":
        if self._started_pwm is not None:
            raise SteeringStateError("front steering servo is already running")
        if self._pwm is None:
            raise SteeringStateError(
                "front steering servo has no PWM output; build it from the "
                "vehicle profile (config factory) before starting"
            )
        output = self._pwm
        output.start()
        self._started_pwm = output
        try:
            self.center()
        except BaseException:
            self._started_pwm = None
            raise
        return self

    def __enter__(self) -> "FrontSteeringServo":
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def set_angle(self, angle_rad: float) -> SteeringCommand:
        command = make_steering_command(angle_rad, self.calibration)
        self.apply(command)
        return command

    def set_yaw(
        self, direction: YawDirection, magnitude_rad: float
    ) -> SteeringCommand:
        command = yaw_to_steering_command(
            direction, magnitude_rad, self.calibration
        )
        self.apply(command)
        return command

    def center(self) -> SteeringCommand:
        return self.set_angle(0.0)

    def apply(self, command: SteeringCommand) -> None:
        if not isinstance(command, SteeringCommand):
            raise TypeError("command must be a SteeringCommand")
        if self._started_pwm is None:
            raise SteeringStateError("front steering servo is not running")
        self._started_pwm.set_pulse_us(command.pulse_us)
        self._command = command

    def disable(self) -> None:
        """Release servo holding torque; normally only use during maintenance."""

        if self._started_pwm is None:
            raise SteeringStateError("front steering servo is not running")
        self._started_pwm.disable()

    def close(self) -> None:
        if self._started_pwm is None:
            return
        try:
            self.center()
        finally:
            self._started_pwm = None
