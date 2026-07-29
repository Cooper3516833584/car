"""Hardware-free tests for the C10B battery telemetry and low-voltage alarm."""

from __future__ import annotations

import unittest

from components.battery_voltage_monitor import (
    BatteryVoltageSample,
    C10BTelemetryError,
    C10BTelemetryParser,
    LowBatteryMonitor,
    decode_stock_c10b_voltage_v,
)


def telemetry_frame(*, voltage_cv: int, motor_disabled: bool = False) -> bytes:
    raw = (int(voltage_cv) * 1000) & 0xFFFF
    frame = bytearray(24)
    frame[0] = 0x7B
    frame[1] = int(motor_disabled)
    frame[20] = raw >> 8
    frame[21] = raw & 0xFF
    checksum = 0
    for value in frame[:22]:
        checksum ^= value
    frame[22] = checksum
    frame[23] = 0x7D
    return bytes(frame)


class _Source:
    def __init__(self, voltages: list[float]) -> None:
        self._samples = [
            BatteryVoltageSample(voltage, 0, float(index))
            for index, voltage in enumerate(voltages)
        ]

    def read_sample(self, timeout_s: float = 1.5) -> BatteryVoltageSample:
        return self._samples.pop(0)


class _Alarm:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def on(self) -> None:
        self.calls.append("on")

    def off(self) -> None:
        self.calls.append("off")


class C10BBatteryTelemetryTests(unittest.TestCase):
    def test_decodes_stock_firmware_wrapped_centivolts(self) -> None:
        raw = (1_093 * 1000) & 0xFFFF
        self.assertEqual(10.93, decode_stock_c10b_voltage_v(raw))

    def test_rejects_unrepresentable_voltage_field(self) -> None:
        with self.assertRaises(C10BTelemetryError):
            decode_stock_c10b_voltage_v(0x1234)

    def test_parser_resynchronizes_and_validates_checksum(self) -> None:
        parser = C10BTelemetryParser(clock=lambda: 42.0)
        invalid = bytearray(telemetry_frame(voltage_cv=1099))
        invalid[22] ^= 0x01
        frames = parser.feed(b"noise" + invalid + telemetry_frame(voltage_cv=1102))
        self.assertEqual(1, len(frames))
        self.assertFalse(frames[0].motor_disabled)
        self.assertEqual((1_102 * 1000) & 0xFFFF, frames[0].battery_raw)
        self.assertEqual(42.0, frames[0].received_at_s)

    def test_alarm_requires_five_consecutive_values_strictly_below_threshold(self) -> None:
        alarm = _Alarm()
        monitor = LowBatteryMonitor(
            _Source([10.99, 10.8, 10.7, 10.6, 10.5, 11.0, 10.9]), alarm
        )
        for _ in range(4):
            monitor.poll_once()
        self.assertEqual([], alarm.calls)
        monitor.poll_once()
        self.assertEqual(["on"], alarm.calls)
        self.assertTrue(monitor.alarm_active)
        monitor.poll_once()
        self.assertEqual(["on", "off"], alarm.calls)
        self.assertFalse(monitor.alarm_active)
        self.assertEqual(0, monitor.consecutive_low_count)


if __name__ == "__main__":
    unittest.main()
