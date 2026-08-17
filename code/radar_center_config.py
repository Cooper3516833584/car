"""Legacy radar-centre runtime-state helpers.

The current runtime-state mechanism lives in ``config/runtime_state.py`` and
writes ``runtime/car_state.json`` (path and allowed values configured in the
TOML ``[runtime]`` section).  This module keeps the old function names for
backward compatibility with external scripts and old deployments; it wraps the
new implementation with the default profile settings.
"""

from __future__ import annotations

from pathlib import Path
from typing import Final

from config.loader import ConfigError, load_car_config
from config.models import RuntimeStateConfig
from config.runtime_state import (
    STATE_KEY,
    RuntimeRadarCenterState,
    normalize_radar_center_behind_a_cm as _normalize,
)

__all__ = [
    "CONFIG_FILENAME",
    "DEFAULT_RADAR_CENTER_BEHIND_A_CM",
    "ALLOWED_RADAR_CENTER_BEHIND_A_CM",
    "normalize_radar_center_behind_a_cm",
    "load_radar_center_behind_a_cm",
    "save_radar_center_behind_a_cm",
]

DEFAULT_RADAR_CENTER_BEHIND_A_CM: Final[float] = 20.0
ALLOWED_RADAR_CENTER_BEHIND_A_CM: Final[tuple[float, ...]] = (20.0, 36.5)
CONFIG_FILENAME: Final[str] = "radar_center_config.json"


def _profile_runtime_config() -> RuntimeStateConfig:
    try:
        return load_car_config().runtime
    except ConfigError:
        return RuntimeStateConfig()


def normalize_radar_center_behind_a_cm(value: object) -> float:
    return _normalize(
        value, _profile_runtime_config().allowed_radar_center_behind_a_cm
    )


def load_radar_center_behind_a_cm(config_path: Path | None = None) -> float:
    runtime = _profile_runtime_config()
    state = RuntimeRadarCenterState(runtime)
    if config_path is not None and (
        config_path.name == CONFIG_FILENAME
        or config_path.name == runtime.state_file.split("/")[-1]
    ):
        state.state_path = Path(config_path)
    return state.load(DEFAULT_RADAR_CENTER_BEHIND_A_CM)


def save_radar_center_behind_a_cm(
    config_path: Path | None, value: object
) -> float:
    runtime = _profile_runtime_config()
    state = RuntimeRadarCenterState(runtime)
    if config_path is not None:
        state.state_path = Path(config_path)
    return state.save(value)
