#!/usr/bin/env python3
"""Send a safe A2/A3 gimbal test command to a patched WHEELTEC C10B board.

This uses the L150 11-byte USB serial frame.  It requires the paired temporary
firmware patch in ``c10b_a2_a3_gimbal.patch``; the stock L150 firmware ignores
the two added gimbal fields.  The command never requests base movement.
"""

from __future__ import annotations

import argparse
import os
import struct
import time

try:
    import termios
except ModuleNotFoundError:  # Lets --help and static checks run on Windows hosts.
    termios = None


HEADER = 0x7B
TAIL = 0x7D
SAFE_MIN_US = 900
SAFE_MAX_US = 2100


def bounded_pulse(value: int) -> int:
    if not SAFE_MIN_US <= value <= SAFE_MAX_US:
        raise argparse.ArgumentTypeError(
            f"pulse width must be {SAFE_MIN_US}..{SAFE_MAX_US} microseconds"
        )
    return value


def frame(a2_us: int, a3_us: int) -> bytes:
    """Build the compatible 11-byte WHEELTEC velocity command frame.

    Bytes 1..2 carry A2 and bytes 5..6 carry A3 only in the patched firmware.
    X and Z velocity remain zero, so the chassis is not commanded to move.
    """
    packet = bytearray(11)
    packet[0] = HEADER
    packet[1:3] = struct.pack(">H", a2_us)
    packet[3:5] = b"\x00\x00"  # X velocity = 0
    packet[5:7] = struct.pack(">H", a3_us)
    packet[7:9] = b"\x00\x00"  # Z angular velocity = 0
    checksum = 0
    for value in packet[:9]:
        checksum ^= value
    packet[9] = checksum
    packet[10] = TAIL
    return bytes(packet)


def configure_serial(device: str) -> int:
    if termios is None:
        raise RuntimeError("this direct serial test program must run on Linux")
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
    settings = termios.tcgetattr(fd)
    settings[0] = termios.IGNPAR
    settings[1] = 0
    settings[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
    settings[3] = 0
    settings[4] = termios.B115200
    settings[5] = termios.B115200
    settings[6][termios.VMIN] = 0
    settings[6][termios.VTIME] = 5
    termios.tcflush(fd, termios.TCIOFLUSH)
    termios.tcsetattr(fd, termios.TCSANOW, settings)
    return fd


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--a2", type=bounded_pulse, default=1500, help="pan pulse width")
    parser.add_argument("--a3", type=bounded_pulse, default=1500, help="tilt pulse width")
    parser.add_argument("--seconds", type=float, default=1.5)
    args = parser.parse_args()
    if not 0.1 <= args.seconds <= 10:
        parser.error("--seconds must be from 0.1 to 10")

    command = frame(args.a2, args.a3)
    print("TX:", command.hex(" "))
    fd = configure_serial(args.device)
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            os.write(fd, command)
            time.sleep(0.05)  # 20 Hz, enough to keep the command responsive
    finally:
        os.close(fd)
    print(f"A2={args.a2} us, A3={args.a3} us sent; base velocity stayed zero.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
