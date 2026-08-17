"""On-site adjustable runtime state (not vehicle static calibration).

The serial screen may switch the radar centre distance behind A between the
allowed values.  The selection is stored as JSON in ``runtime/car_state.json``
(configured via ``[runtime] state_file``) and takes precedence over the TOML
default while remaining a separate runtime file: the program never rewrites
the vehicle TOML profile.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Final

from .models import RuntimeStateConfig

# Backward-compatible legacy state file kept for older deployments that used
# ``radar_center_config.json``; new deployments use ``runtime/car_state.json``.
LEGACY_STATE_FILENAME: Final[str] = "radar_center_config.json"

STATE_KEY: Final[str] = "radar_center_behind_a_cm"


def normalize_radar_center_behind_a_cm(
    value: object,
    allowed: tuple[float, ...] = (20.0, 36.5),
) -> float:
    """Return the matching allowed distance or raise ``ValueError``."""
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            "radar centre distance must be one of "
            + ", ".join(f"{item:g}" for item in allowed)
        ) from exc
    for candidate in allowed:
        if math.isclose(number, float(candidate), rel_tol=0.0, abs_tol=1e-9):
            return float(candidate)
    raise ValueError(
        "radar centre distance must be one of "
        + ", ".join(f"{item:g}" for item in allowed)
    )


class RuntimeRadarCenterState:
    """Load/save the runtime radar-centre selection for one profile."""

    def __init__(
        self,
        config: RuntimeStateConfig,
        *,
        base_directory: str | os.PathLike[str] | None = None,
    ) -> None:
        self.config = config
        state_path = Path(config.state_file)
        if not state_path.is_absolute() and base_directory is not None:
            state_path = Path(base_directory) / state_path
        self.state_path = state_path

    def load(self, default_cm: float) -> float:
        """Return the stored selection, or ``default_cm`` when absent/invalid."""
        try:
            document = json.loads(self.state_path.read_text(encoding="utf-8"))
            return normalize_radar_center_behind_a_cm(
                document[STATE_KEY],
                self.config.allowed_radar_center_behind_a_cm,
            )
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
            json.JSONDecodeError,
        ):
            return float(default_cm)

    def save(self, value: object) -> float:
        """Atomically persist one allowed selection and return it."""
        selected = normalize_radar_center_behind_a_cm(
            value, self.config.allowed_radar_center_behind_a_cm
        )
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.state_path.with_name(
            self.state_path.name + ".tmp"
        )
        payload = {STATE_KEY: selected}
        try:
            with temporary_path.open("w", encoding="utf-8", newline="\n") as stream:
                json.dump(payload, stream, ensure_ascii=True, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_path, self.state_path)
        finally:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass
        return selected


def load_runtime_radar_center_cm(
    config: RuntimeStateConfig,
    default_cm: float,
    *,
    base_directory: str | os.PathLike[str] | None = None,
) -> float:
    """Convenience: load the runtime radar-centre selection."""
    if not config.enabled:
        return float(default_cm)
    return RuntimeRadarCenterState(
        config, base_directory=base_directory
    ).load(default_cm)


def save_runtime_radar_center_cm(
    config: RuntimeStateConfig,
    value: object,
    *,
    base_directory: str | os.PathLike[str] | None = None,
) -> float:
    """Convenience: persist one allowed runtime radar-centre selection."""
    return RuntimeRadarCenterState(
        config, base_directory=base_directory
    ).save(value)
