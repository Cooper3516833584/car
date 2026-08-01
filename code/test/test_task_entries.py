import unittest
from unittest.mock import patch

import main_task1
import main_task2
from components.competition_track import CompetitionTrackSpeedProfile
from main_radar_camera_line_following import build_argument_parser


class TaskEntryTests(unittest.TestCase):
    def test_each_task_has_an_independent_segment_profile(self):
        self.assertEqual(
            main_task1.TASK1_SPEED_PROFILE,
            CompetitionTrackSpeedProfile(8.0, 15.0, 20.0, 15.0),
        )
        self.assertEqual(
            main_task2.TASK2_SPEED_PROFILE,
            CompetitionTrackSpeedProfile(15.0, 15.0, 6.0, 15.0),
        )
        self.assertEqual(
            15.0,
            main_task2.TASK2_CD_SPEED_AFTER_RETAKEOFF_CM_S,
        )
        self.assertIsNot(
            main_task1.TASK1_SPEED_PROFILE,
            main_task2.TASK2_SPEED_PROFILE,
        )

    def test_each_entry_passes_its_profile_to_the_shared_core(self):
        for module, profile in (
            (main_task1, main_task1.TASK1_SPEED_PROFILE),
            (main_task2, main_task2.TASK2_SPEED_PROFILE),
        ):
            core_argv = module.build_core_argv([])
            args = build_argument_parser().parse_args(core_argv)
            self.assertEqual(
                (
                    args.ab_speed_cm_s,
                    args.bc_speed_cm_s,
                    args.cd_speed_cm_s,
                    args.da_speed_cm_s,
                ),
                (
                    profile.ab_cm_s,
                    profile.bc_cm_s,
                    profile.cd_cm_s,
                    profile.da_cm_s,
                ),
            )
        task1_args = build_argument_parser().parse_args(
            main_task1.build_core_argv([])
        )
        task2_args = build_argument_parser().parse_args(
            main_task2.build_core_argv([])
        )
        self.assertTrue(task1_args.wait_for_fleet_start)
        self.assertEqual(13, task1_args.fleet_mission_request_state)
        self.assertEqual(1.0, task1_args.completion_alarm_seconds)
        self.assertTrue(task2_args.wait_for_fleet_start)
        self.assertEqual(14, task2_args.fleet_mission_request_state)
        self.assertEqual(1.0, task2_args.completion_alarm_seconds)
        self.assertEqual(15.0, task2_args.cd_second_speed_cm_s)

    def test_explicit_cli_speed_overrides_task_default(self):
        args = build_argument_parser().parse_args(
            main_task2.build_core_argv(
                ["--ab-speed-cm-s", "18", "--log-level", "DEBUG"]
            )
        )
        self.assertEqual(args.ab_speed_cm_s, 18.0)
        self.assertEqual(args.log_level, "DEBUG")

    def test_entry_calls_shared_core_without_running_hardware_on_import(self):
        with patch.object(main_task1, "_run_core", return_value=7) as run_core:
            result = main_task1.main(["--no-fleet-position"])
        self.assertEqual(result, 7)
        run_core.assert_called_once_with(
            main_task1.build_core_argv(["--no-fleet-position"])
        )


if __name__ == "__main__":
    unittest.main()
