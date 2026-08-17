import shutil
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch

import main_task1
import main_task2
from components.competition_track import CompetitionTrackSpeedProfile
from config.loader import load_car_config
from main_radar_camera_line_following import build_argument_parser

REPO_ROOT = Path(__file__).resolve().parents[2]
REAL_CONFIG = REPO_ROOT / "configs" / "cooper_rock5a_l150.toml"


class TaskEntryTests(unittest.TestCase):
    def test_each_task_has_an_independent_segment_profile(self):
        missions = load_car_config().missions

        self.assertEqual(
            CompetitionTrackSpeedProfile(
                missions.task1.ab_speed_cm_s,
                missions.task1.bc_speed_cm_s,
                missions.task1.cd_speed_cm_s,
                missions.task1.da_speed_cm_s,
            ),
            CompetitionTrackSpeedProfile(8.0, 15.0, 20.0, 15.0),
        )
        self.assertEqual(
            CompetitionTrackSpeedProfile(
                missions.task2.ab_speed_cm_s,
                missions.task2.bc_speed_cm_s,
                missions.task2.cd_speed_before_retakeoff_cm_s,
                missions.task2.da_speed_cm_s,
            ),
            CompetitionTrackSpeedProfile(25.0, 9.0, 4.0, 30.0),
        )
        self.assertEqual(
            30.0,
            missions.task2.cd_speed_after_retakeoff_cm_s,
        )

    def test_each_entry_builds_cli_args_from_its_profile(self):
        missions = load_car_config().missions
        for module, section in (
            (main_task1, missions.task1),
            (main_task2, missions.task2),
        ):
            args = build_argument_parser().parse_args(module.build_core_argv([]))
            cd_before = (
                section.cd_speed_cm_s
                if section is missions.task1
                else section.cd_speed_before_retakeoff_cm_s
            )
            self.assertEqual(
                (
                    args.ab_speed_cm_s,
                    args.bc_speed_cm_s,
                    args.cd_speed_cm_s,
                    args.da_speed_cm_s,
                ),
                (
                    section.ab_speed_cm_s,
                    section.bc_speed_cm_s,
                    cd_before,
                    section.da_speed_cm_s,
                ),
            )
            self.assertTrue(args.wait_for_fleet_start)
            self.assertEqual(
                section.fleet_mission_request_state,
                args.fleet_mission_request_state,
            )
            self.assertEqual(
                section.completion_alarm_seconds,
                args.completion_alarm_seconds,
            )
        task2_args = build_argument_parser().parse_args(
            main_task2.build_core_argv([])
        )
        self.assertEqual(
            missions.task2.cd_speed_after_retakeoff_cm_s,
            task2_args.cd_second_speed_cm_s,
        )

    def test_explicit_cli_speed_overrides_task_default(self):
        args = build_argument_parser().parse_args(
            main_task2.build_core_argv(
                ["--ab-speed-cm-s", "18", "--log-level", "DEBUG"]
            )
        )
        self.assertEqual(args.ab_speed_cm_s, 18.0)
        self.assertEqual(args.log_level, "DEBUG")

    def test_build_core_argv_uses_config_flagged_profile(self):
        # An explicit --config must select that profile's task parameters
        # instead of silently injecting the Cooper defaults.
        tmp = (
            Path(__file__).resolve().parent / f"_hal_tmp_{uuid.uuid4().hex}"
        )
        tmp.mkdir()
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        other = tmp / "other_school.toml"
        content = REAL_CONFIG.read_text(encoding="utf-8").replace(
            "ab_speed_cm_s = 8.0", "ab_speed_cm_s = 99.0"
        )
        other.write_text(content, encoding="utf-8")

        args = build_argument_parser().parse_args(
            main_task1.build_core_argv(["--config", str(other)])
        )
        self.assertEqual(99.0, args.ab_speed_cm_s)
        self.assertEqual(str(other), args.config)

        task2_args = build_argument_parser().parse_args(
            main_task2.build_core_argv(["--config", str(other)])
        )
        # task2 AB default is 25.0 in the Cooper profile; other_school.toml
        # only changed task1 AB, so task2 must still match its own profile.
        self.assertEqual(25.0, task2_args.ab_speed_cm_s)

    def test_entry_delegates_to_shared_core_without_running_hardware(self):
        with patch.object(main_task1, "_run_mission", return_value=7) as run:
            result = main_task1.main(["--no-fleet-position"])
        self.assertEqual(result, 7)
        run.assert_called_once_with("task1", ["--no-fleet-position"])

    def test_task2_delegates_to_shared_core_with_task_name(self):
        with patch.object(main_task2, "_run_mission", return_value=3) as run:
            result = main_task2.main([])
        self.assertEqual(result, 3)
        run.assert_called_once_with("task2", [])


if __name__ == "__main__":
    unittest.main()
