#!/usr/bin/env python3
"""One-shot timed drive: forward, 90-degree right arc, forward.

This is an open-loop real-car test.  It estimates distance from command speed
and elapsed time, so it must only be used in a clear test area with an operator
ready to cut power.  ``--execute`` is deliberately required before it opens any
hardware device.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import select
import sys
import time

try:
    import termios
except ModuleNotFoundError:  # Allows dry-run planning checks on Windows.
    termios = None

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components import AckermannDrive, MotorDirection, RearMotorDriver, plan_ackermann_motion


FORWARD_DISTANCE_MM = 2000.0
TURN_RADIUS_MM = 1000.0
TURN_DEGREES = 90.0
COMMAND_REFRESH_S = 0.05
PHYSICAL_WHEELBASE_MM = 142.5
PHYSICAL_TRACK_WIDTH_MM = 117.1


def confirm_c10b_motor_enabled(device: str = "/dev/ttyACM0") -> None:
    """Require a C10B telemetry frame whose second byte reports KEY2 enabled."""

    if termios is None:
        raise RuntimeError("C10B telemetry probing requires Linux termios")
    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        settings = termios.tcgetattr(fd)
        settings[0] = 0
        settings[1] = 0
        settings[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
        settings[3] = 0
        settings[4] = termios.B115200
        settings[5] = termios.B115200
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        termios.tcflush(fd, termios.TCIFLUSH)

        data = bytearray()
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            readable, _, _ = select.select([fd], [], [], 0.1)
            if readable:
                data.extend(os.read(fd, 256))
            for index in range(len(data) - 1):
                if data[index] == 0x7B:
                    enabled = data[index + 1] == 0x00
                    print(
                        "C10B telemetry KEY2 byte=0x%02X (%s)"
                        % (data[index + 1], "enabled" if enabled else "disabled")
                    )
                    if enabled:
                        return
                    raise RuntimeError("C10B KEY2 reports motor disabled")
        raise RuntimeError("no C10B telemetry frame received; refusing to move")
    finally:
        os.close(fd)


def right_steering_for_radius(radius_mm: float) -> float:
    """Return right steering whose rear-centre turn radius is ``radius_mm``."""

    if radius_mm <= PHYSICAL_TRACK_WIDTH_MM / 2.0:
        raise ValueError("turn radius is too small for the car geometry")
    return -math.atan(
        PHYSICAL_WHEELBASE_MM
        / (radius_mm - PHYSICAL_TRACK_WIDTH_MM / 2.0)
    )


def refresh_motion(
    drive: AckermannDrive,
    *,
    speed_mm_s: float,
    steering_rad: float,
    duration_s: float,
) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        drive.set_motion(
            speed_mm_s,
            steering_rad,
            direction=MotorDirection.FORWARD,
            rear_differential_linked=True,
        )
        time.sleep(COMMAND_REFRESH_S)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="allow real hardware motion")
    parser.add_argument("--forward-only", action="store_true", help="run only the first 2 m forward segment")
    parser.add_argument("--speed-mm-s", type=float, default=1000.0)
    parser.add_argument("--settle-seconds", type=float, default=0.3)
    args = parser.parse_args()
    if not 100.0 <= args.speed_mm_s <= 1200.0:
        parser.error("--speed-mm-s must be in 100..1200")
    if not 0.0 <= args.settle_seconds <= 2.0:
        parser.error("--settle-seconds must be in 0..2")

    right_steering_rad = right_steering_for_radius(TURN_RADIUS_MM)
    turn_plan = plan_ackermann_motion(
        args.speed_mm_s,
        right_steering_rad,
        max_wheel_speed_mm_s=1200.0,
    )
    assert turn_plan.turn_radius_mm is not None
    forward_seconds = FORWARD_DISTANCE_MM / args.speed_mm_s
    turn_seconds = (
        abs(turn_plan.turn_radius_mm)
        * math.radians(TURN_DEGREES)
        / args.speed_mm_s
    )
    print(
        "plan: forward %.2fs, right %.2fs at %.3frad (radius %.0fmm), forward %.2fs"
        % (
            forward_seconds,
            turn_seconds,
            right_steering_rad,
            abs(turn_plan.turn_radius_mm),
            forward_seconds,
        )
    )
    if not args.execute:
        print("dry run only; pass --execute to open PWM0 and /dev/ttyACM0")
        return 0

    confirm_c10b_motor_enabled()
    rear = RearMotorDriver(
        max_wheel_speed_mm_s=1200.0,
        command_timeout_s=0.35,
    )
    with AckermannDrive(rear_motors=rear) as drive:
        print("segment 1: forward")
        refresh_motion(
            drive,
            speed_mm_s=args.speed_mm_s,
            steering_rad=0.0,
            duration_s=forward_seconds,
        )
        drive.stop(center_steering=True)
        if args.forward_only:
            print("complete: forward-only; motors stopped and steering centered")
            return 0
        time.sleep(args.settle_seconds)

        print("segment 2: right 90-degree arc")
        refresh_motion(
            drive,
            speed_mm_s=args.speed_mm_s,
            steering_rad=right_steering_rad,
            duration_s=turn_seconds,
        )
        drive.stop(center_steering=True)
        time.sleep(args.settle_seconds)

        print("segment 3: forward")
        refresh_motion(
            drive,
            speed_mm_s=args.speed_mm_s,
            steering_rad=0.0,
            duration_s=forward_seconds,
        )
        drive.stop(center_steering=True)
    print("complete: motors stopped and steering centered")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
