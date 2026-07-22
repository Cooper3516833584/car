#!/usr/bin/env python3
"""Calibrated front-steering servo on ROCK 5A physical Pin 23 (PWM0_M2)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
import time
from typing import Final


PWM_PERIOD_NS: Final[int] = 20_000_000  # 50 Hz
SERVO_MIN_US: Final[int] = 800
SERVO_MAX_US: Final[int] = 2200
STEERING_DIRECTION_SIGN: Final[float] = -1.0
STEERING_RIGHT_MAX_RAD: Final[float] = -0.32
STEERING_LEFT_MAX_RAD: Final[float] = 0.49
CALIBRATION_MIN_RAD: Final[float] = -0.49
CALIBRATION_MAX_RAD: Final[float] = 0.32
FACTORY_CENTER_US: Final[int] = 1501
STEERING_CENTER_US: Final[int] = 1580


class SteeringStateError(RuntimeError):
    """The steering PWM component is not started or cannot access PWM0."""


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


def _finite_angle(angle_rad: float) -> float:
    angle = float(angle_rad)
    if not math.isfinite(angle):
        raise ValueError("angle_rad must be finite")
    if not STEERING_RIGHT_MAX_RAD <= angle <= STEERING_LEFT_MAX_RAD:
        raise ValueError(
            "angle_rad must be in "
            f"[{STEERING_RIGHT_MAX_RAD}, {STEERING_LEFT_MAX_RAD}]"
        )
    return angle


def steering_angle_to_pulse_us(angle_rad: float) -> int:
    """Apply the WHEELTEC L150 cubic steering-angle calibration.

    Vehicle steering remains positive/left and negative/right.  Real-car logs
    plus direct wheel observation confirmed that this servo/linkage is mounted
    opposite to the factory curve: logical left therefore evaluates the curve
    at a negative calibration angle and produces a pulse above centre.
    """

    angle = _finite_angle(angle_rad)
    calibration_angle = STEERING_DIRECTION_SIGN * angle
    if not CALIBRATION_MIN_RAD <= calibration_angle <= CALIBRATION_MAX_RAD:
        raise ValueError("logical steering angle exceeds calibration travel")
    servo_angle = (
        -0.628 * calibration_angle**3
        + 1.269 * calibration_angle**2
        - 1.772 * calibration_angle
        + 1.573
    )
    factory_pulse = 1500.0 + (servo_angle - 1.572) * 640.62
    pulse = factory_pulse + (STEERING_CENTER_US - FACTORY_CENTER_US)
    return round(max(SERVO_MIN_US, min(SERVO_MAX_US, pulse)))


def make_steering_command(angle_rad: float) -> SteeringCommand:
    angle = _finite_angle(angle_rad)
    return SteeringCommand(angle, steering_angle_to_pulse_us(angle))


def yaw_to_steering_command(
    direction: YawDirection, magnitude_rad: float
) -> SteeringCommand:
    """Build a command from an explicit yaw direction and angle magnitude."""

    if not isinstance(direction, YawDirection):
        raise TypeError("direction must be a YawDirection")
    magnitude = float(magnitude_rad)
    if not math.isfinite(magnitude) or magnitude < 0.0:
        raise ValueError("magnitude_rad must be finite and non-negative")
    return make_steering_command(direction.value * magnitude)


def _write(path: Path, value: int | str) -> None:
    path.write_text(f"{value}\n", encoding="ascii")


def _find_pwm0_chip() -> Path:
    for chip in Path("/sys/class/pwm").glob("pwmchip*"):
        if "fd8b0000.pwm" in str(chip.resolve()):
            return chip
    raise SteeringStateError(
        "PWM0 is unavailable; enable rk3588-pwm0-m2 with rsetup and reboot"
    )


class FrontSteeringServo:
    """Sysfs-PWM controller for the front steering servo.

    The caller normally runs as root on the ROCK 5A.  Starting centres and
    enables the servo.  Closing returns to centre and deliberately leaves PWM
    enabled so the front wheels continue to hold the safe centre position.
    """

    def __init__(self, pwm_chip: str | Path | None = None) -> None:
        self._configured_chip = Path(pwm_chip) if pwm_chip is not None else None
        self._pwm: Path | None = None
        self._command = make_steering_command(0.0)

    @property
    def command(self) -> SteeringCommand:
        return self._command

    @property
    def is_running(self) -> bool:
        return self._pwm is not None

    def start(self) -> "FrontSteeringServo":
        if self._pwm is not None:
            raise SteeringStateError("front steering servo is already running")
        chip = self._configured_chip or _find_pwm0_chip()
        pwm = chip / "pwm0"
        if not pwm.exists():
            _write(chip / "export", 0)
            for _ in range(20):
                if pwm.exists():
                    break
                time.sleep(0.05)
        if not pwm.exists():
            raise SteeringStateError("PWM0 export did not create pwm0")
        enable = pwm / "enable"
        if enable.read_text(encoding="ascii").strip() == "1":
            _write(enable, 0)
        _write(pwm / "period", PWM_PERIOD_NS)
        _write(pwm / "polarity", "normal")
        self._pwm = pwm
        try:
            self.center()
        except BaseException:
            self._pwm = None
            raise
        return self

    def __enter__(self) -> "FrontSteeringServo":
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def set_angle(self, angle_rad: float) -> SteeringCommand:
        command = make_steering_command(angle_rad)
        self.apply(command)
        return command

    def set_yaw(
        self, direction: YawDirection, magnitude_rad: float
    ) -> SteeringCommand:
        command = yaw_to_steering_command(direction, magnitude_rad)
        self.apply(command)
        return command

    def center(self) -> SteeringCommand:
        return self.set_angle(0.0)

    def apply(self, command: SteeringCommand) -> None:
        if not isinstance(command, SteeringCommand):
            raise TypeError("command must be a SteeringCommand")
        if self._pwm is None:
            raise SteeringStateError("front steering servo is not running")
        _write(self._pwm / "duty_cycle", command.pulse_us * 1000)
        _write(self._pwm / "enable", 1)
        self._command = command

    def disable(self) -> None:
        """Release servo holding torque; normally only use during maintenance."""

        if self._pwm is None:
            raise SteeringStateError("front steering servo is not running")
        _write(self._pwm / "enable", 0)

    def close(self) -> None:
        if self._pwm is None:
            return
        try:
            self.center()
        finally:
            self._pwm = None
