"""Independent D-task runner with a per-segment black-loop speed profile."""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import sys

from components import (
    CompetitionTrackFollower,
    CompetitionTrackSpeedProfile,
    RadarMount,
)
from main_fixed_track_test import (
    CompetitionCarApplication,
    DEFAULT_D500_PORT,
    LOG,
    MainConfig,
    RADAR_CENTER_BEHIND_A_ALONG_AB_CM,
    configure_logging,
    default_log_dir,
    shutdown_logging,
)


class CompetitionTaskApplication(CompetitionCarApplication):
    """Use the tested calibration lifecycle with task-specific loop speeds."""

    def __init__(
        self,
        config: MainConfig,
        speed_profile: CompetitionTrackSpeedProfile,
    ) -> None:
        super().__init__(config)
        self.speed_profile = speed_profile
        self.follower = CompetitionTrackFollower(
            drive=self.drive,
            track=self.track,
            speed_profile=speed_profile,
            on_state_changed=self._on_follower_state,
        )
        self._follower_state = self.follower.state


def build_argument_parser(
    *,
    task_name: str,
    default_speed_profile: CompetitionTrackSpeedProfile,
) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=f"D-task {task_name}: follow the black loop once from A."
    )
    parser.add_argument("--radar-port", default=DEFAULT_D500_PORT)
    parser.add_argument("--radar-x-cm", type=float, default=0.0)
    parser.add_argument("--radar-y-cm", type=float, default=0.0)
    parser.add_argument("--radar-yaw-cw-deg", type=float, default=0.0)
    parser.add_argument("--startup-scans", type=int, default=3)
    parser.add_argument("--calibration-timeout", type=float, default=30.0)
    parser.add_argument(
        "--radar-center-behind-a-cm",
        type=float,
        default=RADAR_CENTER_BEHIND_A_ALONG_AB_CM,
    )
    parser.add_argument(
        "--ab-speed-cm-s", type=float, default=default_speed_profile.ab_cm_s
    )
    parser.add_argument(
        "--bc-speed-cm-s", type=float, default=default_speed_profile.bc_cm_s
    )
    parser.add_argument(
        "--cd-speed-cm-s", type=float, default=default_speed_profile.cd_cm_s
    )
    parser.add_argument(
        "--da-speed-cm-s", type=float, default=default_speed_profile.da_cm_s
    )
    parser.add_argument(
        "--log-level",
        choices=("OFF", "DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
    )
    parser.add_argument("--log-dir", default=None)
    return parser


def main(
    argv: list[str] | None = None,
    *,
    task_name: str,
    default_speed_profile: CompetitionTrackSpeedProfile,
) -> int:
    args = build_argument_parser(
        task_name=task_name,
        default_speed_profile=default_speed_profile,
    ).parse_args(argv)
    requested_log_dir = (
        default_log_dir() if args.log_dir is None else Path(args.log_dir)
    )
    try:
        configure_logging(requested_log_dir, args.log_level)
    except OSError as exc:
        print(
            f"cannot create detailed log in {requested_log_dir}: {exc}",
            file=sys.stderr,
        )
        return 2

    app: CompetitionTaskApplication | None = None
    try:
        speed_profile = CompetitionTrackSpeedProfile(
            args.ab_speed_cm_s,
            args.bc_speed_cm_s,
            args.cd_speed_cm_s,
            args.da_speed_cm_s,
        )
        app = CompetitionTaskApplication(
            MainConfig(
                radar_port=args.radar_port,
                radar_mount=RadarMount(
                    args.radar_x_cm,
                    args.radar_y_cm,
                    args.radar_yaw_cw_deg,
                ),
                startup_scan_count=args.startup_scans,
                calibration_timeout_s=args.calibration_timeout,
                radar_center_behind_a_cm=args.radar_center_behind_a_cm,
                speed_cm_s=speed_profile.max_speed_cm_s,
            ),
            speed_profile,
        )

        def stop_handler(signum, _frame) -> None:
            LOG.info("received signal %s; stopping", signum)
            app.request_stop()

        signal.signal(signal.SIGINT, stop_handler)
        signal.signal(signal.SIGTERM, stop_handler)
        app.run()
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception:
        LOG.exception("D-task car runner failed")
        return 1
    finally:
        if app is not None:
            app.close()
        shutdown_logging()
