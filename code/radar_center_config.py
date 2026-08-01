"""Persistent selection for the radar centre's distance behind point A."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Final


DEFAULT_RADAR_CENTER_BEHIND_A_CM: Final[float] = 20.0
ALLOWED_RADAR_CENTER_BEHIND_A_CM: Final[tuple[float, ...]] = (20.0, 36.5)
CONFIG_FILENAME: Final[str] = "radar_center_config.json"


def normalize_radar_center_behind_a_cm(value: object) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("radar centre distance must be 20 or 36.5 cm") from exc
    for allowed in ALLOWED_RADAR_CENTER_BEHIND_A_CM:
        if math.isclose(number, allowed, rel_tol=0.0, abs_tol=1e-9):
            return allowed
    raise ValueError("radar centre distance must be 20 or 36.5 cm")


def load_radar_center_behind_a_cm(config_path: Path) -> float:
    try:
        document = json.loads(config_path.read_text(encoding="utf-8"))
        return normalize_radar_center_behind_a_cm(
            document["radar_center_behind_a_cm"]
        )
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return DEFAULT_RADAR_CENTER_BEHIND_A_CM


def save_radar_center_behind_a_cm(config_path: Path, value: object) -> float:
    selected = normalize_radar_center_behind_a_cm(value)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = config_path.with_name(config_path.name + ".tmp")
    payload = {"radar_center_behind_a_cm": selected}
    try:
        with temporary_path.open("w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, config_path)
    finally:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass
    return selected
