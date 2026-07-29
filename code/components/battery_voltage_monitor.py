#!/usr/bin/env python3
"""C10B battery-telemetry reader and persistent low-voltage alarm service.

The WHEELTEC C10B firmware continuously transmits 24-byte telemetry frames on
the same 115200-8N1 port used for velocity commands.  Its battery field is
stored at bytes 20--21.  The supplied L150 firmware writes the centivolt value
multiplied by 1000 into a 16-bit field, so the reader reverses that documented
modulo-16-bit encoding instead of treating the field as a normal millivolt
value.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import logging
import os
import select
import threading
import time
from typing import Callable, Final, Protocol

from .rear_motor import DEFAULT_DEVICE
from .sound_light_alarm import SoundLightAlarm

try:
    import termios
except ModuleNotFoundError:  # Allows parser/controller tests on Windows.
    termios = None


LOG = logging.getLogger(__name__)

TELEMETRY_HEADER: Final[int] = 0x7B
TELEMETRY_TAIL: Final[int] = 0x7D
TELEMETRY_LENGTH: Final[int] = 24
TELEMETRY_CHECKSUM_INDEX: Final[int] = 22
BATTERY_HIGH_INDEX: Final[int] = 20
BATTERY_LOW_INDEX: Final[int] = 21
DEFAULT_BATTERY_MIN_VOLTAGE_V: Final[float] = 6.0
DEFAULT_BATTERY_MAX_VOLTAGE_V: Final[float] = 18.0


class C10BTelemetryError(RuntimeError):
    """The C10B telemetry stream cannot be opened or decoded safely."""


@dataclass(frozen=True, slots=True)
class C10BTelemetryFrame:
    """One checksum-verified 24-byte C10B telemetry frame."""

    motor_disabled: bool
    battery_raw: int
    received_at_s: float


@dataclass(frozen=True, slots=True)
class BatteryVoltageSample:
    """One decoded C10B battery reading."""

    voltage_v: float
    raw_value: int
    received_at_s: float


def _xor_checksum(data: bytes) -> int:
    checksum = 0
    for value in data:
        checksum ^= value
    return checksum


def decode_stock_c10b_voltage_v(
    raw_value: int,
    *,
    min_voltage_v: float = DEFAULT_BATTERY_MIN_VOLTAGE_V,
    max_voltage_v: float = DEFAULT_BATTERY_MAX_VOLTAGE_V,
) -> float:
    """Decode the battery field written by the supplied L150 C10B firmware.

    ``Get_Voltage()`` produces centivolts, and ``bluetooth.c`` then multiplies
    that value by 1000 before assigning it to a 16-bit ``short``.  We recover
    the only centivolt candidate in the configured physical range whose low
    16 bits reproduce the received value.  A non-matching value is rejected
    instead of being misreported as a plausible but incorrect voltage.
    """

    raw = int(raw_value)
    if not 0 <= raw <= 0xFFFF:
        raise ValueError("raw_value must fit in an unsigned 16-bit field")
    minimum_cv = round(float(min_voltage_v) * 100.0)
    maximum_cv = round(float(max_voltage_v) * 100.0)
    if not 0 < minimum_cv <= maximum_cv:
        raise ValueError("battery voltage range must be positive and ordered")

    matches = [
        centivolts
        for centivolts in range(minimum_cv, maximum_cv + 1)
        if (centivolts * 1000) & 0xFFFF == raw
    ]
    if len(matches) != 1:
        raise C10BTelemetryError(
            "battery field 0x%04X does not uniquely match the stock C10B "
            "voltage encoding in %.2f..%.2f V" % (raw, min_voltage_v, max_voltage_v)
        )
    return matches[0] / 100.0


class C10BTelemetryParser:
    """Incrementally parse and re-synchronise C10B telemetry frames."""

    def __init__(self, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._buffer = bytearray()
        self._clock = clock

    def feed(self, data: bytes) -> list[C10BTelemetryFrame]:
        self._buffer.extend(data)
        frames: list[C10BTelemetryFrame] = []
        while True:
            header_index = self._buffer.find(TELEMETRY_HEADER)
            if header_index < 0:
                self._buffer.clear()
                break
            if header_index:
                del self._buffer[:header_index]
            if len(self._buffer) < TELEMETRY_LENGTH:
                break

            candidate = self._buffer[:TELEMETRY_LENGTH]
            if (
                candidate[-1] != TELEMETRY_TAIL
                or _xor_checksum(candidate[:TELEMETRY_CHECKSUM_INDEX])
                != candidate[TELEMETRY_CHECKSUM_INDEX]
            ):
                del self._buffer[0]
                continue

            frames.append(
                C10BTelemetryFrame(
                    motor_disabled=candidate[1] != 0,
                    battery_raw=(candidate[BATTERY_HIGH_INDEX] << 8)
                    | candidate[BATTERY_LOW_INDEX],
                    received_at_s=self._clock(),
                )
            )
            del self._buffer[:TELEMETRY_LENGTH]
        return frames


def _open_c10b_telemetry_port(device: str) -> int:
    if termios is None:
        raise C10BTelemetryError("C10B telemetry requires Linux termios")
    try:
        fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as exc:
        raise C10BTelemetryError(f"cannot open C10B telemetry port {device}: {exc}") from exc
    try:
        settings = termios.tcgetattr(fd)
        settings[0] = termios.IGNPAR
        settings[1] = 0
        settings[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
        settings[3] = 0
        settings[4] = termios.B115200
        settings[5] = termios.B115200
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        return fd
    except BaseException:
        os.close(fd)
        raise


class C10BBatteryVoltageReader:
    """Read checksum-verified battery samples from C10B telemetry only.

    This opens the port read-only and never flushes or writes it, so it can run
    alongside the rear-motor command writer.
    """

    def __init__(
        self,
        device: str = DEFAULT_DEVICE,
        *,
        min_voltage_v: float = DEFAULT_BATTERY_MIN_VOLTAGE_V,
        max_voltage_v: float = DEFAULT_BATTERY_MAX_VOLTAGE_V,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.device = device
        self.min_voltage_v = float(min_voltage_v)
        self.max_voltage_v = float(max_voltage_v)
        if not 0 < self.min_voltage_v <= self.max_voltage_v:
            raise ValueError("battery voltage range must be positive and ordered")
        self._clock = clock
        self._parser = C10BTelemetryParser(clock=clock)
        self._fd: int | None = None

    def start(self) -> "C10BBatteryVoltageReader":
        if self._fd is not None:
            raise C10BTelemetryError("C10B battery reader is already running")
        self._fd = _open_c10b_telemetry_port(self.device)
        return self

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def __enter__(self) -> "C10BBatteryVoltageReader":
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def read_sample(self, timeout_s: float = 1.5) -> BatteryVoltageSample:
        if self._fd is None:
            raise C10BTelemetryError("C10B battery reader is not running")
        timeout = float(timeout_s)
        if timeout <= 0.0:
            raise ValueError("timeout_s must be positive")
        deadline = self._clock() + timeout
        while True:
            remaining = deadline - self._clock()
            if remaining <= 0.0:
                raise C10BTelemetryError(
                    f"no valid C10B telemetry frame received from {self.device} within {timeout:g} s"
                )
            readable, _, _ = select.select([self._fd], [], [], remaining)
            if not readable:
                continue
            try:
                data = os.read(self._fd, 256)
            except BlockingIOError:
                continue
            if not data:
                continue
            frames = self._parser.feed(data)
            if not frames:
                continue
            frame = frames[-1]
            return BatteryVoltageSample(
                voltage_v=decode_stock_c10b_voltage_v(
                    frame.battery_raw,
                    min_voltage_v=self.min_voltage_v,
                    max_voltage_v=self.max_voltage_v,
                ),
                raw_value=frame.battery_raw,
                received_at_s=frame.received_at_s,
            )


class BatterySampleSource(Protocol):
    def read_sample(self, timeout_s: float = 1.5) -> BatteryVoltageSample: ...


class AlarmOutput(Protocol):
    def on(self) -> None: ...

    def off(self) -> None: ...


class LowBatteryMonitor:
    """Alarm after five consecutive low C10B voltage readings."""

    def __init__(
        self,
        sample_source: BatterySampleSource,
        alarm: AlarmOutput,
        *,
        threshold_v: float = 11.0,
        required_consecutive_low: int = 5,
        read_timeout_s: float = 1.5,
    ) -> None:
        self._sample_source = sample_source
        self._alarm = alarm
        self.threshold_v = float(threshold_v)
        self.required_consecutive_low = int(required_consecutive_low)
        self.read_timeout_s = float(read_timeout_s)
        if self.threshold_v <= 0.0:
            raise ValueError("threshold_v must be positive")
        if self.required_consecutive_low <= 0:
            raise ValueError("required_consecutive_low must be positive")
        if self.read_timeout_s <= 0.0:
            raise ValueError("read_timeout_s must be positive")
        self.consecutive_low_count = 0
        self.alarm_active = False

    def poll_once(self) -> BatteryVoltageSample:
        sample = self._sample_source.read_sample(self.read_timeout_s)
        if sample.voltage_v < self.threshold_v:
            self.consecutive_low_count += 1
            if (
                self.consecutive_low_count >= self.required_consecutive_low
                and not self.alarm_active
            ):
                self._alarm.on()
                self.alarm_active = True
                LOG.error(
                    "battery voltage %.2f V has been below %.2f V for %d consecutive readings; alarm enabled",
                    sample.voltage_v,
                    self.threshold_v,
                    self.consecutive_low_count,
                )
        else:
            if self.alarm_active:
                self._alarm.off()
                LOG.info("battery voltage recovered to %.2f V; alarm silenced", sample.voltage_v)
            self.consecutive_low_count = 0
            self.alarm_active = False
        return sample

    def run_forever(
        self,
        *,
        period_s: float = 2.0,
        stop_event: threading.Event | None = None,
    ) -> None:
        period = float(period_s)
        if period <= 0.0:
            raise ValueError("period_s must be positive")
        while stop_event is None or not stop_event.is_set():
            started_at = time.monotonic()
            try:
                sample = self.poll_once()
                LOG.info(
                    "C10B battery %.2f V (raw=0x%04X, consecutive_low=%d)",
                    sample.voltage_v,
                    sample.raw_value,
                    self.consecutive_low_count,
                )
            except C10BTelemetryError as exc:
                LOG.warning("C10B battery read failed: %s", exc)
            remaining = period - (time.monotonic() - started_at)
            if remaining > 0.0 and stop_event is not None:
                stop_event.wait(remaining)
            elif remaining > 0.0:
                time.sleep(remaining)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--threshold-v", type=float, default=11.0)
    parser.add_argument("--period-s", type=float, default=2.0)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    alarm = SoundLightAlarm().initialize(active=False)
    with C10BBatteryVoltageReader(args.device) as reader:
        LowBatteryMonitor(reader, alarm, threshold_v=args.threshold_v).run_forever(
            period_s=args.period_s
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
