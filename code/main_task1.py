#!/usr/bin/env python3
"""D-task 1: continuously follow the black loop from A for one lap."""

from __future__ import annotations

from components.competition_track import CompetitionTrackSpeedProfile
from competition_task_runtime import main as _run_task


# Competition preset. Keep four independent values so the lap can be tuned
# before deployment without changing task 2's program.
TASK1_SPEED_PROFILE = CompetitionTrackSpeedProfile(10.0, 10.0, 10.0, 10.0)


def main(argv: list[str] | None = None) -> int:
    return _run_task(
        argv,
        task_name="task 1 (payload drop)",
        default_speed_profile=TASK1_SPEED_PROFILE,
    )


if __name__ == "__main__":
    raise SystemExit(main())
