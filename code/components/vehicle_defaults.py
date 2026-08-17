"""Backward-compatible vehicle defaults (single source for components).

These constants equal the verified Cooper ROCK 5A + WHEELTEC L150 profile and
exist so existing tests and legacy call sites can construct components without
a TOML file.  The competition program never relies on them: the composition
root builds every component from the TOML profile, and the profile file under
``configs/`` is the single source of truth for the running car.
"""

from __future__ import annotations

from typing import Final

DEFAULT_WHEELBASE_MM: Final[float] = 142.5
DEFAULT_PHYSICAL_TRACK_WIDTH_MM: Final[float] = 117.1
# C10B firmware-compiled track used by the serial protocol:
# Vz = (right - left) / firmware_track.  Deliberately different from the
# physical track width; the two must never be merged.
DEFAULT_FIRMWARE_TRACK_WIDTH_MM: Final[float] = 164.0
DEFAULT_BODY_LENGTH_MM: Final[float] = 230.0
DEFAULT_BODY_WIDTH_MM: Final[float] = 145.0
DEFAULT_WHEEL_THICKNESS_MM: Final[float] = 26.4
DEFAULT_OUTER_WHEEL_WIDTH_MM: Final[float] = 143.5
DEFAULT_REAR_AXLE_TO_BODY_CENTER_MM: Final[float] = 71.25
DEFAULT_MIN_TURN_RADIUS_MM: Final[float] = 350.0
