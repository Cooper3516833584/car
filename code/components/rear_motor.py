#!/usr/bin/env python3
"""Rear-motor component for the WHEELTEC L150 C10B USB driver board.

The installed Ackermann firmware accepts chassis linear velocity and angular
velocity, not two raw motor values.  This module exposes linked and independent
wheel APIs, converts feasible left/right targets to that protocol, and rejects
targets the firmware cannot reproduce because of its 350 mm minimum turn
radius.  Positive wheel speed means vehicle-forward; negative means reverse.

The driver continuously refreshes commands at 20 Hz.  If the caller stops
refreshing the target, a watchdog sends zero velocity.  Closing the component
also sends several zero frames before releasing the serial device.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import os
import struct
import threading
import time
from typing import Final

try:
    import termios
except ModuleNotFoundError:  # Allows frame/conversion unit tests on Windows.
    termios = None


FRAME_HEADER: Final[int] = 0x7B
FRAME_TAIL: Final[int] = 0x7D
FRAME_LENGTH: Final[int] = 11
DEFAULT_DEVICE: Final[str] = "/dev/ttyACM0"
DEFAULT_TRACK_WIDTH_MM: Final[float] = 164.0
DEFAULT_MIN_TURN_RADIUS_MM: Final[float] = 350.0
FIRMWARE_MAX_LINEAR_MM_S: Final[float] = 1200.0


class UnsupportedWheelCommand(ValueError):
    """The requested wheel targets cannot be represented by this firmware."""


class DriverStateError(RuntimeError):
    """The motor driver is not running or its writer thread has failed."""


class MotorDirection(Enum):
    """Direction used by :meth:`RearMotorDriver.set_linked`."""

    FORWARD = 1
    REVERSE = -1


@dataclass(frozen=True, slots=True)
class WheelSpeeds:
    """Signed left and right rear-wheel targets in millimetres per second."""

    left_mm_s: float
    right_mm_s: float


@dataclass(frozen=True, slots=True)
class ChassisCommand:
    """Quantized values carried in one C10B 11-byte command frame."""

    linear_mm_s: int
    angular_mrad_s: int
    requested: WheelSpeeds

    def represented_wheel_speeds(
        self, track_width_mm: float = DEFAULT_TRACK_WIDTH_MM
    ) -> WheelSpeeds:
        """Return wheel speeds represented after integer protocol quantization."""

        angular_rad_s = self.angular_mrad_s / 1000.0
        half_delta = angular_rad_s * track_width_mm / 2.0
        return WheelSpeeds(
            self.linear_mm_s - half_delta,
            self.linear_mm_s + half_delta,
        )


STOP_COMMAND: Final[ChassisCommand] = ChassisCommand(0, 0, WheelSpeeds(0.0, 0.0))


def _finite_number(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def wheel_speeds_to_chassis(
    left_mm_s: float,
    right_mm_s: float,
    *,
    max_wheel_speed_mm_s: float = 300.0,
    track_width_mm: float = DEFAULT_TRACK_WIDTH_MM,
    min_turn_radius_mm: float = DEFAULT_MIN_TURN_RADIUS_MM,
) -> ChassisCommand:
    """Convert two rear-wheel targets to a feasible C10B Ackermann command.

    Equal signed speeds are linked straight-line motion.  Different speeds are
    converted with ``Vx=(left+right)/2`` and
    ``Vz=(right-left)/track_width``.  The stock firmware clamps turns tighter
    than 350 mm, so this function rejects such targets instead of silently
    producing different wheel speeds than requested.
    """

    left = _finite_number("left_mm_s", left_mm_s)
    right = _finite_number("right_mm_s", right_mm_s)
    max_speed = _finite_number("max_wheel_speed_mm_s", max_wheel_speed_mm_s)
    track = _finite_number("track_width_mm", track_width_mm)
    min_radius = _finite_number("min_turn_radius_mm", min_turn_radius_mm)
    if not 0.0 < max_speed <= FIRMWARE_MAX_LINEAR_MM_S:
        raise ValueError("max_wheel_speed_mm_s must be in (0, 1200]")
    if track <= 0.0 or min_radius <= 0.0:
        raise ValueError("track_width_mm and min_turn_radius_mm must be positive")
    if abs(left) > max_speed or abs(right) > max_speed:
        raise ValueError(
            f"wheel speed exceeds configured limit +/-{max_speed:g} mm/s"
        )

    requested = WheelSpeeds(left, right)
    linear = (left + right) / 2.0
    angular_rad_s = (right - left) / track

    if math.isclose(linear, 0.0, abs_tol=1e-9):
        if math.isclose(angular_rad_s, 0.0, abs_tol=1e-9):
            return STOP_COMMAND
        raise UnsupportedWheelCommand(
            "the Ackermann firmware cannot rotate in place or drive the rear "
            "wheels in opposite directions"
        )

    if not math.isclose(angular_rad_s, 0.0, abs_tol=1e-12):
        radius_mm = abs(linear / angular_rad_s)
        if radius_mm + 1e-9 < min_radius:
            raise UnsupportedWheelCommand(
                f"requested turn radius {radius_mm:.1f} mm is below the "
                f"firmware minimum {min_radius:.1f} mm"
            )

    linear_int = round(linear)
    angular_mrad_int = round(angular_rad_s * 1000.0)
    if not -32768 <= linear_int <= 32767 or not -32768 <= angular_mrad_int <= 32767:
        raise ValueError("command exceeds the signed 16-bit serial protocol")
    return ChassisCommand(linear_int, angular_mrad_int, requested)


def build_velocity_frame(command: ChassisCommand) -> bytes:
    """Build the verified 11-byte C10B velocity frame with XOR checksum."""

    frame = bytearray(FRAME_LENGTH)
    frame[0] = FRAME_HEADER
    struct.pack_into(">h", frame, 3, command.linear_mm_s)
    struct.pack_into(">h", frame, 7, command.angular_mrad_s)
    checksum = 0
    for value in frame[:9]:
        checksum ^= value
    frame[9] = checksum
    frame[10] = FRAME_TAIL
    return bytes(frame)


def _open_c10b_serial(device: str) -> int:
    if termios is None:
        raise RuntimeError("C10B serial I/O must run on Linux (the ROCK 5A)")
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
    try:
        settings = termios.tcgetattr(fd)
        settings[0] = termios.IGNPAR
        settings[1] = 0
        settings[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
        settings[3] = 0
        settings[4] = termios.B115200
        settings[5] = termios.B115200
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 1
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd
    except BaseException:
        os.close(fd)
        raise


class RearMotorDriver:
    """Threaded, fail-safe controller for the two C10B rear motors.

    Call a target-setting method more frequently than ``command_timeout_s``.
    The most recent target is refreshed at ``send_rate_hz`` until the watchdog
    expires.  Use this class as a context manager so every exit sends stop
    frames, including exception paths.
    """

    def __init__(
        self,
        device: str = DEFAULT_DEVICE,
        *,
        max_wheel_speed_mm_s: float = 300.0,
        send_rate_hz: float = 20.0,
        command_timeout_s: float = 0.5,
        track_width_mm: float = DEFAULT_TRACK_WIDTH_MM,
        min_turn_radius_mm: float = DEFAULT_MIN_TURN_RADIUS_MM,
        stop_frame_count: int = 5,
    ) -> None:
        self.device = device
        self.max_wheel_speed_mm_s = _finite_number(
            "max_wheel_speed_mm_s", max_wheel_speed_mm_s
        )
        self.send_rate_hz = _finite_number("send_rate_hz", send_rate_hz)
        self.command_timeout_s = _finite_number(
            "command_timeout_s", command_timeout_s
        )
        self.track_width_mm = _finite_number("track_width_mm", track_width_mm)
        self.min_turn_radius_mm = _finite_number(
            "min_turn_radius_mm", min_turn_radius_mm
        )
        if not 1.0 <= self.send_rate_hz <= 100.0:
            raise ValueError("send_rate_hz must be in [1, 100]")
        if not 0.1 <= self.command_timeout_s <= 10.0:
            raise ValueError("command_timeout_s must be in [0.1, 10]")
        if not 1 <= stop_frame_count <= 20:
            raise ValueError("stop_frame_count must be in [1, 20]")
        # Validate all geometry and configured speed limits once.
        wheel_speeds_to_chassis(
            0.0,
            0.0,
            max_wheel_speed_mm_s=self.max_wheel_speed_mm_s,
            track_width_mm=self.track_width_mm,
            min_turn_radius_mm=self.min_turn_radius_mm,
        )
        self.stop_frame_count = stop_frame_count
        self._fd: int | None = None
        self._command = STOP_COMMAND
        self._last_update = 0.0
        self._state_lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._writer: threading.Thread | None = None
        self._writer_error: BaseException | None = None

    @property
    def requested_wheel_speeds(self) -> WheelSpeeds:
        with self._state_lock:
            return self._command.requested

    @property
    def is_running(self) -> bool:
        with self._state_lock:
            return self._fd is not None and self._writer_error is None

    def start(self) -> "RearMotorDriver":
        with self._state_lock:
            if self._fd is not None:
                raise DriverStateError("rear motor driver is already running")
            fd = _open_c10b_serial(self.device)
            self._fd = fd
            self._command = STOP_COMMAND
            self._last_update = time.monotonic()
            self._writer_error = None
            self._shutdown.clear()
            self._writer = threading.Thread(
                target=self._writer_loop,
                name="c10b-rear-motor-writer",
                daemon=True,
            )
            self._writer.start()
        return self

    def __enter__(self) -> "RearMotorDriver":
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def set_linked(
        self,
        speed_mm_s: float,
        direction: MotorDirection = MotorDirection.FORWARD,
    ) -> ChassisCommand:
        """Run both rear motors together at one non-negative speed."""

        speed = _finite_number("speed_mm_s", speed_mm_s)
        if speed < 0.0:
            raise ValueError("speed_mm_s must be non-negative; use direction")
        if not isinstance(direction, MotorDirection):
            raise TypeError("direction must be a MotorDirection")
        signed_speed = speed * direction.value
        return self.set_wheels(signed_speed, signed_speed)

    def set_wheels(self, left_mm_s: float, right_mm_s: float) -> ChassisCommand:
        """Atomically set signed left/right speeds when the firmware can do so."""

        command = wheel_speeds_to_chassis(
            left_mm_s,
            right_mm_s,
            max_wheel_speed_mm_s=self.max_wheel_speed_mm_s,
            track_width_mm=self.track_width_mm,
            min_turn_radius_mm=self.min_turn_radius_mm,
        )
        self._set_command(command)
        return command

    def set_left(self, speed_mm_s: float) -> ChassisCommand:
        """Change only the left target, retaining the current right target."""

        with self._state_lock:
            right = self._command.requested.right_mm_s
        return self.set_wheels(speed_mm_s, right)

    def set_right(self, speed_mm_s: float) -> ChassisCommand:
        """Change only the right target, retaining the current left target."""

        with self._state_lock:
            left = self._command.requested.left_mm_s
        return self.set_wheels(left, speed_mm_s)

    def stop(self) -> None:
        """Immediately command both motors to zero while keeping the port open."""

        self._set_command(STOP_COMMAND)
        self._write_command(STOP_COMMAND)

    def close(self) -> None:
        """Stop the writer, send repeated zero frames, and close the serial port."""

        with self._state_lock:
            fd = self._fd
            writer = self._writer
            self._command = STOP_COMMAND
            self._last_update = time.monotonic()
        if fd is None:
            return
        self._shutdown.set()
        if writer is not None and writer is not threading.current_thread():
            writer.join(timeout=2.0)
        try:
            frame = build_velocity_frame(STOP_COMMAND)
            for _ in range(self.stop_frame_count):
                with self._io_lock:
                    os.write(fd, frame)
                time.sleep(0.05)
        finally:
            os.close(fd)
            with self._state_lock:
                self._fd = None
                self._writer = None

    def _set_command(self, command: ChassisCommand) -> None:
        with self._state_lock:
            self._raise_if_unavailable_locked()
            self._command = command
            self._last_update = time.monotonic()

    def _raise_if_unavailable_locked(self) -> None:
        if self._fd is None:
            raise DriverStateError("rear motor driver is not running")
        if self._writer_error is not None:
            raise DriverStateError("rear motor writer failed") from self._writer_error

    def _write_command(self, command: ChassisCommand) -> None:
        frame = build_velocity_frame(command)
        with self._state_lock:
            self._raise_if_unavailable_locked()
            fd = self._fd
        assert fd is not None
        with self._io_lock:
            os.write(fd, frame)

    def _writer_loop(self) -> None:
        period = 1.0 / self.send_rate_hz
        next_send = time.monotonic()
        try:
            while not self._shutdown.is_set():
                wait_s = max(0.0, next_send - time.monotonic())
                if self._shutdown.wait(wait_s):
                    break
                now = time.monotonic()
                with self._state_lock:
                    command = (
                        self._command
                        if now - self._last_update <= self.command_timeout_s
                        else STOP_COMMAND
                    )
                self._write_command(command)
                next_send = time.monotonic() + period
        except BaseException as exc:
            with self._state_lock:
                self._writer_error = exc
            self._shutdown.set()
