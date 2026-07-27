#!/usr/bin/env python3
"""Active-high sound/light alarm on ROCK 5A GPIO4_B3 (physical Pin 11)."""

from __future__ import annotations

import argparse
import errno
import os
from pathlib import Path
import time
from typing import Final


GPIO_BANK_LABEL: Final[str] = "gpio4"
GPIO_BANK_OFFSET: Final[int] = 11  # GPIO4_B3: bank B starts at line 8.
DEFAULT_SYSFS_GPIO_ROOT: Final[Path] = Path("/sys/class/gpio")


class AlarmGPIOError(RuntimeError):
    """GPIO4_B3 cannot be resolved, initialized, or controlled."""


def resolve_gpio_number(
    sysfs_gpio_root: str | Path = DEFAULT_SYSFS_GPIO_ROOT,
    *,
    bank_label: str = GPIO_BANK_LABEL,
    bank_offset: int = GPIO_BANK_OFFSET,
) -> int:
    """Resolve a bank-relative line without assuming gpiochip probe order."""

    root = Path(sysfs_gpio_root)
    for chip in root.glob("gpiochip*"):
        try:
            if (chip / "label").read_text(encoding="ascii").strip() != bank_label:
                continue
            base = int((chip / "base").read_text(encoding="ascii").strip())
            count = int((chip / "ngpio").read_text(encoding="ascii").strip())
        except (OSError, ValueError):
            continue
        if not 0 <= bank_offset < count:
            raise AlarmGPIOError(
                f"{bank_label} offset {bank_offset} is outside its {count} lines"
            )
        return base + bank_offset
    raise AlarmGPIOError(f"GPIO bank {bank_label!r} is unavailable under {root}")


class SoundLightAlarm:
    """Control the high-level-triggered alarm through the GPIO sysfs ABI.

    ``initialize()`` atomically selects output-low, so the alarm is off while
    the pin direction changes.  The pin remains exported and keeps its last
    output after this object is discarded; this is intentional for startup use.
    """

    def __init__(
        self,
        gpio_number: int | None = None,
        *,
        sysfs_gpio_root: str | Path = DEFAULT_SYSFS_GPIO_ROOT,
    ) -> None:
        self._root = Path(sysfs_gpio_root)
        self._gpio_number = (
            resolve_gpio_number(self._root)
            if gpio_number is None
            else int(gpio_number)
        )
        if self._gpio_number < 0:
            raise ValueError("gpio_number must be non-negative")
        self._gpio = self._root / f"gpio{self._gpio_number}"

    @property
    def gpio_number(self) -> int:
        return self._gpio_number

    @property
    def is_initialized(self) -> bool:
        return (self._gpio / "value").exists()

    @property
    def is_active(self) -> bool:
        self._require_initialized()
        try:
            return (self._gpio / "value").read_text(encoding="ascii").strip() == "1"
        except OSError as exc:
            raise AlarmGPIOError(f"cannot read GPIO {self._gpio_number}: {exc}") from exc

    def initialize(self, *, active: bool = False) -> "SoundLightAlarm":
        """Export GPIO4_B3 and select output-low before any optional alarm-on."""

        if not self._gpio.exists():
            try:
                self._write(self._root / "export", self._gpio_number)
            except OSError as exc:
                if exc.errno != errno.EBUSY:
                    raise AlarmGPIOError(
                        f"cannot export GPIO {self._gpio_number}: {exc}"
                    ) from exc
            for _ in range(50):
                if self._gpio.exists():
                    break
                time.sleep(0.01)
        if not self._gpio.exists():
            raise AlarmGPIOError(f"export did not create {self._gpio}")

        # The sysfs "low" direction value makes output selection and the safe
        # inactive level one operation, avoiding a short high pulse.
        try:
            self._write(self._gpio / "direction", "low")
        except OSError as exc:
            raise AlarmGPIOError(
                f"cannot configure GPIO {self._gpio_number} as output-low: {exc}"
            ) from exc
        if active:
            self.on()
        return self

    def on(self) -> None:
        """Enable sound and light by driving the active-high input high."""

        self._set_raw_value(1)

    def off(self) -> None:
        """Silence the alarm by driving its input low."""

        self._set_raw_value(0)

    def set_active(self, active: bool) -> None:
        self._set_raw_value(1 if active else 0)

    def grant_group_access(self, group: str = "gpio") -> None:
        """Allow members of *group* to control the initialized output."""

        self._require_initialized()
        try:
            import grp

            gid = grp.getgrnam(group).gr_gid
            for name in ("direction", "value"):
                path = self._gpio / name
                os.chown(path, 0, gid)
                os.chmod(path, 0o664)
        except (KeyError, OSError) as exc:
            raise AlarmGPIOError(
                f"cannot grant GPIO {self._gpio_number} access to group {group!r}: {exc}"
            ) from exc

    def _set_raw_value(self, value: int) -> None:
        self._require_initialized()
        try:
            self._write(self._gpio / "value", value)
        except OSError as exc:
            raise AlarmGPIOError(f"cannot write GPIO {self._gpio_number}: {exc}") from exc

    def _require_initialized(self) -> None:
        if not self.is_initialized:
            raise AlarmGPIOError("alarm GPIO is not initialized; call initialize() first")

    @staticmethod
    def _write(path: Path, value: int | str) -> None:
        path.write_text(f"{value}\n", encoding="ascii")


def alarm_on() -> SoundLightAlarm:
    """Convenience entry point: initialize if needed and enable the alarm."""

    alarm = SoundLightAlarm()
    if not alarm.is_initialized:
        alarm.initialize()
    alarm.on()
    return alarm


def alarm_off() -> SoundLightAlarm:
    """Convenience entry point: initialize if needed and silence the alarm."""

    alarm = SoundLightAlarm()
    if not alarm.is_initialized:
        alarm.initialize()
    alarm.off()
    return alarm


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    state = parser.add_mutually_exclusive_group()
    state.add_argument("--on", action="store_true", help="drive high and enable alarm")
    state.add_argument("--off", action="store_true", help="drive low and silence alarm")
    parser.add_argument(
        "--grant-group",
        metavar="GROUP",
        help="chown direction/value to this group and grant group writes",
    )
    args = parser.parse_args()

    alarm = SoundLightAlarm().initialize(active=args.on)
    if args.off or not args.on:
        alarm.off()
    if args.grant_group:
        alarm.grant_group_access(args.grant_group)
    print(
        f"GPIO{alarm.gpio_number} GPIO4_B3 alarm "
        f"{'ON (high)' if alarm.is_active else 'OFF (low)'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
