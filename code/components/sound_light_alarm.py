#!/usr/bin/env python3
"""Sound/light alarm as a simple switchable binary output.

This module only expresses "turn the sound/light alarm on or off".  The
board-specific GPIO details (sysfs root, bank label, line offset, active-low
polarity) come from the TOML profile through the GPIO HAL layer.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from hal.gpio import (
    DigitalOutput,
    GPIOBackendError,
    LinuxSysfsBankGPIOOutput,
    resolve_gpio_number,
)

__all__ = [
    "AlarmGPIOError",
    "SoundLightAlarm",
    "alarm_off",
    "alarm_on",
    "resolve_gpio_number",
]

# Legacy module constants kept for existing call sites; the production entry
# builds the alarm from the TOML [hardware.alarm_gpio] section.
GPIO_BANK_LABEL: str = "gpio4"
GPIO_BANK_OFFSET: int = 11
DEFAULT_SYSFS_GPIO_ROOT: Path = Path("/sys/class/gpio")


class AlarmGPIOError(RuntimeError):
    """The alarm GPIO line cannot be resolved, initialized, or controlled."""


class SoundLightAlarm:
    """Control the sound/light alarm through a configured digital output.

    ``initialize()`` atomically selects output-high, so an active-low alarm
    stays off while the pin direction changes.  The pin remains exported and
    keeps its last output after this object is discarded; this is intentional
    for startup use.
    """

    def __init__(
        self,
        gpio_number: int | None = None,
        *,
        sysfs_gpio_root: str | Path = DEFAULT_SYSFS_GPIO_ROOT,
        bank_label: str = GPIO_BANK_LABEL,
        line_offset: int = GPIO_BANK_OFFSET,
        active_low: bool = True,
        output: DigitalOutput | None = None,
    ) -> None:
        if output is not None:
            self._output = output
        else:
            try:
                self._output = LinuxSysfsBankGPIOOutput(
                    sysfs_root=sysfs_gpio_root,
                    bank_label=bank_label,
                    line_offset=line_offset,
                    active_low=active_low,
                    gpio_number=gpio_number,
                )
            except GPIOBackendError as exc:
                raise AlarmGPIOError(str(exc)) from exc
        self._active_low = bool(active_low)

    @property
    def gpio_number(self) -> int:
        return int(getattr(self._output, "gpio_number", -1))

    @property
    def is_initialized(self) -> bool:
        return bool(getattr(self._output, "is_initialized", False))

    @property
    def is_active(self) -> bool:
        return bool(getattr(self._output, "is_active", False))

    def initialize(self, *, active: bool = False) -> "SoundLightAlarm":
        """Export the configured line and select the safe inactive level."""
        try:
            self._output.initialize(active=active)
        except GPIOBackendError as exc:
            raise AlarmGPIOError(str(exc)) from exc
        return self

    def on(self) -> None:
        """Enable sound and light."""
        self.set_active(True)

    def off(self) -> None:
        """Silence the alarm."""
        self.set_active(False)

    def set_active(self, active: bool) -> None:
        try:
            self._output.set_active(bool(active))
        except GPIOBackendError as exc:
            raise AlarmGPIOError(str(exc)) from exc

    def grant_group_access(self, group: str = "gpio") -> None:
        """Allow members of *group* to control the initialized output.

        Only supported for the sysfs bank backend.
        """

        output = getattr(self, "_output", None)
        grant = getattr(output, "grant_group_access", None)
        if grant is None:
            raise AlarmGPIOError(
                "this GPIO backend does not support group access grants"
            )
        try:
            grant(group)
        except GPIOBackendError as exc:
            raise AlarmGPIOError(str(exc)) from exc


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
    state.add_argument("--on", action="store_true", help="drive active and enable alarm")
    state.add_argument("--off", action="store_true", help="drive inactive and silence alarm")
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
        f"GPIO{alarm.gpio_number} alarm "
        f"{'ON' if alarm.is_active else 'OFF'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
