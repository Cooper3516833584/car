#!/usr/bin/env python3
"""Click-to-run open-loop test: drive forward about 1 m, then stop.

This file controls only the C10B rear motors.  It deliberately leaves the
front-servo PWM unchanged, so run ``servo_pulse_test.py`` first when testing a
new straight-ahead pulse.  Distance is estimated from speed x time.
"""

from __future__ import annotations

import os
from pathlib import Path
import select
import sys
import time

try:
    import termios
except ModuleNotFoundError:  # Allows syntax/import checks on Windows.
    termios = None

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components import MotorDirection, RearMotorDriver


# Edit these values if a slower/shorter test is needed, then click Run.
SPEED_MM_S = 1000.0
DISTANCE_MM = 1000.0
COMMAND_REFRESH_S = 0.05


def confirm_c10b_motor_enabled(device: str = "/dev/ttyACM0") -> None:
    if termios is None:
        raise RuntimeError("this real-car test requires Linux termios")
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
                if data[index] != 0x7B:
                    continue
                status = data[index + 1]
                print(f"C10B telemetry KEY2 byte=0x{status:02X}")
                if status == 0x00:
                    return
                raise RuntimeError("C10B KEY2 reports motor disabled")
        raise RuntimeError("no C10B telemetry frame received; refusing to move")
    finally:
        os.close(fd)


def main() -> int:
    if not 100.0 <= SPEED_MM_S <= 1200.0:
        raise ValueError("SPEED_MM_S must be in 100..1200")
    if not 100.0 <= DISTANCE_MM <= 5000.0:
        raise ValueError("DISTANCE_MM must be in 100..5000")
    duration_s = DISTANCE_MM / SPEED_MM_S
    print(
        f"rear motors only: {SPEED_MM_S:.0f} mm/s for {duration_s:.2f} s "
        f"(~{DISTANCE_MM:.0f} mm)"
    )
    confirm_c10b_motor_enabled()
    with RearMotorDriver(
        max_wheel_speed_mm_s=1200.0,
        command_timeout_s=0.35,
    ) as rear:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rear.set_linked(SPEED_MM_S, MotorDirection.FORWARD)
            time.sleep(COMMAND_REFRESH_S)
        rear.stop()
    print("complete: rear motors stopped; front-servo PWM was not changed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
