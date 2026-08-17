#!/usr/bin/env python3
"""D-task 2 one-key entry using the shared radar+camera line follower.

This entry only selects the task name and delegates to the shared core.  The
task speeds (including the CD retakeoff switch), mission-request state and
completion alarm duration come from the TOML profile
(``[missions.task2]``); explicit CLI arguments still win.
"""

from __future__ import annotations

import sys

from config.loader import load_car_config
from main_radar_camera_line_following import run_mission as _run_mission


def build_core_argv(argv: list[str] | None = None) -> list[str]:
    """Legacy CLI helper mirroring the default profile for task 2."""

    forwarded = list(sys.argv[1:] if argv is None else argv)
    task = load_car_config().missions.task2
    return [
        "--wait-for-fleet-start",
        "--fleet-mission-request-state",
        str(task.fleet_mission_request_state),
        "--completion-alarm-seconds",
        str(task.completion_alarm_seconds),
        "--ab-speed-cm-s",
        str(task.ab_speed_cm_s),
        "--bc-speed-cm-s",
        str(task.bc_speed_cm_s),
        "--cd-speed-cm-s",
        str(task.cd_speed_before_retakeoff_cm_s),
        "--cd-second-speed-cm-s",
        str(task.cd_speed_after_retakeoff_cm_s),
        "--da-speed-cm-s",
        str(task.da_speed_cm_s),
        *forwarded,
    ]


def main(argv: list[str] | None = None) -> int:
    return _run_mission("task2", argv)


if __name__ == "__main__":
    raise SystemExit(main())
