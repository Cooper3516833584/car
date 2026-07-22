#!/usr/bin/env python3
"""Read-only D500 UART health probe; never writes the radar or motor board."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.radar_driver import (  # noqa: E402
    DEFAULT_D500_PORT,
    D500SerialDriver,
    RadarPacket,
    RadarScan,
    RadarScanAssembler,
    RectangleFieldCalibrator,
)
from main import rebase_calibration_to_start_pose  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_D500_PORT)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--fit-map", action="store_true")
    parser.add_argument("--startup-scans", type=int, default=3)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    assembler = RadarScanAssembler()
    packets: list[RadarPacket] = []
    complete_scans: list[RadarScan] = []

    def on_packet(packet: RadarPacket) -> None:
        packets.append(packet)
        complete_scans.extend(assembler.feed(packet))

    driver = D500SerialDriver(port=args.port, on_packet=on_packet)
    started = time.monotonic()
    try:
        driver.start()
        connected = driver.wait_connected(min(2.0, args.seconds))
        deadline = started + args.seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(0.05, remaining))
    finally:
        driver.close()

    stats = driver.parser.stats
    print(
        f"port={args.port} connected={connected} packets={len(packets)} "
        f"complete_scans={len(complete_scans)} crc_errors={stats.crc_errors} "
        f"discarded_bytes={stats.discarded_bytes}"
    )
    if not connected:
        print("FAIL: UART could not be opened", file=sys.stderr)
        return 2
    if not packets:
        print("FAIL: UART opened but no valid 54 2C packets arrived", file=sys.stderr)
        return 3
    if len(complete_scans) < 1:
        print("FAIL: valid packets arrived but no complete revolution was assembled", file=sys.stderr)
        return 4
    if args.fit_map:
        if args.startup_scans <= 0:
            print("FAIL: --startup-scans must be positive", file=sys.stderr)
            return 5
        if len(complete_scans) < args.startup_scans:
            print("FAIL: not enough complete scans for map fitting", file=sys.stderr)
            return 5
        try:
            fitted = RectangleFieldCalibrator().calibrate(
                complete_scans[-args.startup_scans :]
            )
            calibration = rebase_calibration_to_start_pose(fitted)
        except Exception as exc:
            print(f"FAIL: rectangle fitting failed: {exc}", file=sys.stderr)
            return 5
        corners = " ".join(
            f"({x_cm:.1f},{y_cm:.1f})"
            for x_cm, y_cm in calibration.field_polygon_cm
        )
        print(
            "MAP PASS: startup_pose=(0.0,0.0,0.0deg) "
            f"nearest_edge_ccw={calibration.selected_edge_ccw_from_car_deg:.2f}deg "
            f"corners={corners}"
        )
    print("PASS: D500 UART, 54 2C framing, CRC and complete-revolution assembly are working")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
