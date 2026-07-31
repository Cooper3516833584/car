#!/usr/bin/env python3
"""D-task 2 one-key entry using the shared radar+camera line follower.

Only the four task-specific speeds live here.  Steering, radar localization,
camera correction, FleetBus reporting and later tuning are always inherited
from ``main_radar_camera_line_following.py``.
"""

from __future__ import annotations

import sys

from components.competition_track import CompetitionTrackSpeedProfile
from main_radar_camera_line_following import main as _run_core


# Task 2 (dynamic landing) segment speeds.  A faster AB default provides margin
# for the rule requiring the car to reach B within 15 seconds.  The four values
# remain independently adjustable for field timing.
TASK2_AB_SPEED_CM_S = 15.0
TASK2_BC_SPEED_CM_S = 15.0
TASK2_CD_SPEED_CM_S = 6.0
TASK2_DA_SPEED_CM_S = 15.0

TASK2_SPEED_PROFILE = CompetitionTrackSpeedProfile(
    TASK2_AB_SPEED_CM_S,
    TASK2_BC_SPEED_CM_S,
    TASK2_CD_SPEED_CM_S,
    TASK2_DA_SPEED_CM_S,
)


def build_core_argv(argv: list[str] | None = None) -> list[str]:
    """Prepend task defaults while allowing explicit CLI overrides."""

    forwarded = list(sys.argv[1:] if argv is None else argv)
    return [
        "--wait-for-fleet-start",
        "--fleet-mission-request-state",
        "14",
        "--ab-speed-cm-s",
        str(TASK2_AB_SPEED_CM_S),
        "--bc-speed-cm-s",
        str(TASK2_BC_SPEED_CM_S),
        "--cd-speed-cm-s",
        str(TASK2_CD_SPEED_CM_S),
        "--da-speed-cm-s",
        str(TASK2_DA_SPEED_CM_S),
        *forwarded,
    ]


def main(argv: list[str] | None = None) -> int:
    return _run_core(build_core_argv(argv))


if __name__ == "__main__":
    raise SystemExit(main())
