"""Hardware abstraction layer: PWM output.

Only the abstract :class:`PWMOutput` interface and the Linux sysfs
implementation live here.  The steering component never knows about a specific
board's device-tree names; every value (sysfs root, chip match, channel,
period, polarity) comes from the TOML profile via this layer.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Protocol, runtime_checkable


@runtime_checkable
class PWMOutput(Protocol):
    """Minimal PWM interface consumed by the steering servo component."""

    def start(self) -> None: ...

    def set_pulse_us(self, pulse_us: int) -> None: ...

    def disable(self) -> None: ...

    def close(self) -> None: ...


class PWMBackendError(RuntimeError):
    """The configured PWM output cannot be opened or controlled."""


def _write(path: Path, value: int | str) -> None:
    path.write_text(f"{value}\n", encoding="ascii")


class LinuxSysfsPWMOutput:
    """Linux sysfs ``/sys/class/pwm`` PWM output.

    The chip is selected by resolving each ``pwmchip*`` symlink and keeping
    the one whose device path contains ``chip_device_match`` (for example
    ``fd8b0000.pwm`` on the ROCK 5A).  This avoids depending on probe order.
    """

    def __init__(
        self,
        *,
        sysfs_root: str | Path = "/sys/class/pwm",
        chip_device_match: str = "fd8b0000.pwm",
        channel: int = 0,
        period_ns: int = 20_000_000,
        polarity: str = "normal",
        pwm_chip: str | Path | None = None,
    ) -> None:
        if channel < 0:
            raise ValueError("pwm channel cannot be negative")
        if period_ns <= 0:
            raise ValueError("pwm period_ns must be positive")
        if polarity not in ("normal", "inversed"):
            raise ValueError("pwm polarity must be 'normal' or 'inversed'")
        self._root = Path(sysfs_root)
        self._chip_match = chip_device_match
        self._channel = channel
        self._period_ns = period_ns
        self._polarity = polarity
        self._configured_chip = Path(pwm_chip) if pwm_chip is not None else None
        self._pwm: Path | None = None

    @property
    def is_running(self) -> bool:
        return self._pwm is not None

    def _find_chip(self) -> Path:
        if self._configured_chip is not None:
            return self._configured_chip
        for chip in self._root.glob("pwmchip*"):
            try:
                resolved = str(chip.resolve())
            except OSError:
                continue
            if self._chip_match in resolved:
                return chip
        raise PWMBackendError(
            f"cannot find a PWM chip matching {self._chip_match!r} under "
            f"{self._root}; check the device-tree/pinmux overlay for your board"
        )

    def start(self) -> "LinuxSysfsPWMOutput":
        if self._pwm is not None:
            raise PWMBackendError("PWM output is already running")
        chip = self._find_chip()
        pwm = chip / f"pwm{self._channel}"
        if not pwm.exists():
            _write(chip / "export", self._channel)
            for _ in range(20):
                if pwm.exists():
                    break
                time.sleep(0.05)
        if not pwm.exists():
            raise PWMBackendError(
                f"PWM export did not create {pwm}; check permissions"
            )
        enable = pwm / "enable"
        try:
            if enable.read_text(encoding="ascii").strip() == "1":
                _write(enable, 0)
            _write(pwm / "period", self._period_ns)
            _write(pwm / "polarity", self._polarity)
        except OSError as exc:
            raise PWMBackendError(f"cannot configure {pwm}: {exc}") from exc
        self._pwm = pwm
        return self

    def set_pulse_us(self, pulse_us: int) -> None:
        if self._pwm is None:
            raise PWMBackendError("PWM output is not running")
        pulse = int(pulse_us)
        if pulse < 0:
            raise ValueError("pulse_us cannot be negative")
        try:
            _write(self._pwm / "duty_cycle", pulse * 1000)
            _write(self._pwm / "enable", 1)
        except OSError as exc:
            raise PWMBackendError(
                f"cannot write PWM duty cycle for {self._pwm}: {exc}"
            ) from exc

    def disable(self) -> None:
        if self._pwm is None:
            raise PWMBackendError("PWM output is not running")
        try:
            _write(self._pwm / "enable", 0)
        except OSError as exc:
            raise PWMBackendError(
                f"cannot disable PWM output {self._pwm}: {exc}"
            ) from exc

    def close(self) -> None:
        self._pwm = None
