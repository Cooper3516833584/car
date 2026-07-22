#!/usr/bin/env python3
"""Low-speed, time-bounded WHEELTEC L150 wheel test for /dev/ttyACM0.

The program sends the documented 11-byte velocity frame at 20 Hz and always
sends several zero-velocity frames before closing the serial port.
"""

from __future__ import annotations

import argparse
import os
import struct
import time

try:
    import termios
except ModuleNotFoundError:
    termios = None


HEADER = 0x7B
TAIL = 0x7D


def bounded_speed(value: str) -> int:
    speed = int(value)
    if not -300 <= speed <= 300:
        raise argparse.ArgumentTypeError("speed must be in -300..300 mm/s")
    return speed


def make_frame(linear_mm_s: int, angular_mrad_s: int = 0) -> bytes:
    packet = bytearray(11)
    packet[0] = HEADER
    packet[3:5] = struct.pack(">h", linear_mm_s)
    packet[7:9] = struct.pack(">h", angular_mrad_s)
    checksum = 0
    for value in packet[:9]:
        checksum ^= value
    packet[9] = checksum
    packet[10] = TAIL
    return bytes(packet)


def open_serial(device: str) -> int:
    if termios is None:
        raise RuntimeError("this test must run on Linux (the ROCK 5A)")
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
    settings = termios.tcgetattr(fd)
    settings[0] = termios.IGNPAR
    settings[1] = 0
    settings[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
    settings[3] = 0
    settings[4] = termios.B115200
    settings[5] = termios.B115200
    settings[6][termios.VMIN] = 0
    settings[6][termios.VTIME] = 1
    termios.tcflush(fd, termios.TCIOFLUSH)
    termios.tcsetattr(fd, termios.TCSANOW, settings)
    return fd


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--linear-mm-s", type=bounded_speed, default=100)
    parser.add_argument("--seconds", type=float, default=1.0)
    args = parser.parse_args()
    if not 0.2 <= args.seconds <= 5.0:
        parser.error("--seconds must be from 0.2 to 5.0")

    move = make_frame(args.linear_mm_s)
    stop = make_frame(0)
    print("move frame:", move.hex(" "))
    fd = open_serial(args.device)
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            os.write(fd, move)
            time.sleep(0.05)
    finally:
        for _ in range(5):
            os.write(fd, stop)
            time.sleep(0.05)
        os.close(fd)
    print("stop frames sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
