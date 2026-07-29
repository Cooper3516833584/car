#!/usr/bin/env python3
"""D-task 2: continuously follow the black loop from A for one lap."""

from __future__ import annotations

from components.competition_track import CompetitionTrackSpeedProfile
from competition_task_runtime import main as _run_task


# This entry deliberately contains no task-2 timing rule.  It is an
# independently deployable profile; timing remains a measured field result.
TASK2_SPEED_PROFILE = CompetitionTrackSpeedProfile(10.0, 10.0, 10.0, 10.0)


def main(argv: list[str] | None = None) -> int:
    return _run_task(
        argv,
        task_name="task 2 (dynamic landing)",
        default_speed_profile=TASK2_SPEED_PROFILE,
    )


if __name__ == "__main__":
    raise SystemExit(main())
