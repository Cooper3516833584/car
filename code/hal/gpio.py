"""Hardware abstraction layer: digital output (GPIO).

Only the abstract :class:`DigitalOutput` interface and the Linux sysfs
bank/line implementation live here.  The alarm component only expresses
"turn the sound/light device on or off"; board-specific bank labels, line
offsets and active-low polarity come from the TOML profile.
"""

from __future__ import annotations

import errno
import time
from pathlib import Path
from typing import Protocol, runtime_checkable


@runtime_checkable
class DigitalOutput(Protocol):
    """Minimal binary-output interface consumed by the alarm component."""

    def initialize(self, active: bool = False) -> None: ...

    def set_active(self, active: bool) -> None: ...

    def close(self) -> None: ...


class GPIOBackendError(RuntimeError):
    """The configured GPIO line cannot be resolved, initialized or controlled."""


def resolve_gpio_number(
    sysfs_gpio_root: str | Path = "/sys/class/gpio",
    *,
    bank_label: str = "gpio4",
    bank_offset: int = 11,
) -> int:
    """Resolve a bank-relative line without assuming gpiochip probe order.

    The chip whose ``label`` equals ``bank_label`` provides ``base`` and
    ``ngpio``; the global number is ``base + bank_offset``.
    """

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
            raise GPIOBackendError(
                f"{bank_label} offset {bank_offset} is outside its {count} lines"
            )
        return base + bank_offset
    raise GPIOBackendError(f"GPIO bank {bank_label!r} is unavailable under {root}")


def _write(path: Path, value: int | str) -> None:
    path.write_text(f"{value}\n", encoding="ascii")


class LinuxSysfsBankGPIOOutput:
    """Linux sysfs GPIO output selected by bank label + line offset.

    ``initialize()`` atomically selects direction ``high`` so an active-low
    device stays off while the direction changes.  With ``active_low=True`` a
    raw 0 drives the output active (the verified ROCK 5A alarm wiring).
    """

    def __init__(
        self,
        *,
        sysfs_root: str | Path = "/sys/class/gpio",
        bank_label: str = "gpio4",
        line_offset: int = 11,
        active_low: bool = True,
        gpio_number: int | None = None,
    ) -> None:
        if line_offset < 0:
            raise ValueError("line_offset cannot be negative")
        self._root = Path(sysfs_root)
        self._gpio_number = (
            resolve_gpio_number(
                self._root,
                bank_label=bank_label,
                bank_offset=line_offset,
            )
            if gpio_number is None
            else int(gpio_number)
        )
        if self._gpio_number < 0:
            raise ValueError("gpio_number must be non-negative")
        self._active_low = bool(active_low)
        self._gpio = self._root / f"gpio{self._gpio_number}"

    @property
    def gpio_number(self) -> int:
        return self._gpio_number

    @property
    def active_low(self) -> bool:
        return self._active_low

    @property
    def is_initialized(self) -> bool:
        return (self._gpio / "value").exists()

    @property
    def is_active(self) -> bool:
        self._require_initialized()
        try:
            raw = (self._gpio / "value").read_text(encoding="ascii").strip()
        except OSError as exc:
            raise GPIOBackendError(
                f"cannot read GPIO {self._gpio_number}: {exc}"
            ) from exc
        if self._active_low:
            return raw == "0"
        return raw == "1"

    def initialize(self, active: bool = False) -> "LinuxSysfsBankGPIOOutput":
        if not self._gpio.exists():
            try:
                _write(self._root / "export", self._gpio_number)
            except OSError as exc:
                if exc.errno != errno.EBUSY:
                    raise GPIOBackendError(
                        f"cannot export GPIO {self._gpio_number}: {exc}"
                    ) from exc
            for _ in range(50):
                if self._gpio.exists():
                    break
                time.sleep(0.01)
        if not self._gpio.exists():
            raise GPIOBackendError(f"export did not create {self._gpio}")
        try:
            _write(self._gpio / "direction", "high")
        except OSError as exc:
            raise GPIOBackendError(
                f"cannot configure GPIO {self._gpio_number} as output-high: {exc}"
            ) from exc
        if active:
            self.set_active(True)
        return self

    def set_active(self, active: bool) -> None:
        self._require_initialized()
        raw = 0 if (active == self._active_low) else 1
        try:
            _write(self._gpio / "value", raw)
        except OSError as exc:
            raise GPIOBackendError(
                f"cannot write GPIO {self._gpio_number}: {exc}"
            ) from exc

    def grant_group_access(self, group: str = "gpio") -> None:
        """Allow members of *group* to control the initialized output."""

        self._require_initialized()
        try:
            import grp

            gid = grp.getgrnam(group).gr_gid
            for name in ("direction", "value"):
                path = self._gpio / name
                import os

                os.chown(path, 0, gid)
                os.chmod(path, 0o664)
        except (KeyError, OSError) as exc:
            raise GPIOBackendError(
                f"cannot grant GPIO {self._gpio_number} access to group "
                f"{group!r}: {exc}"
            ) from exc

    def close(self) -> None:
        # Keep the exported line at its last level, matching the legacy
        # startup behaviour of the alarm service.
        return None

    def _require_initialized(self) -> None:
        if not self.is_initialized:
            raise GPIOBackendError(
                "GPIO output is not initialized; call initialize() first"
            )
