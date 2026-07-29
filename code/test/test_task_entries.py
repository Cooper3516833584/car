import unittest
from pathlib import Path

import main_task1
import main_task2
from competition_task_runtime import build_argument_parser
from components.competition_track import CompetitionTrackSpeedProfile


class TaskEntryTests(unittest.TestCase):
    def test_each_task_has_a_separate_ten_cm_s_profile(self):
        self.assertEqual(
            main_task1.TASK1_SPEED_PROFILE,
            CompetitionTrackSpeedProfile.uniform(10.0),
        )
        self.assertEqual(
            main_task2.TASK2_SPEED_PROFILE,
            CompetitionTrackSpeedProfile.uniform(10.0),
        )
        self.assertIsNot(
            main_task1.TASK1_SPEED_PROFILE,
            main_task2.TASK2_SPEED_PROFILE,
        )

    def test_task_two_has_no_embedded_arrival_timing_requirement(self):
        source = Path(main_task2.__file__).read_text(encoding="utf-8")
        self.assertNotIn("15.0", source)
        self.assertNotIn("90.0", source)

    def test_each_entry_exposes_independent_segment_speed_arguments(self):
        for name, profile in (
            ("task 1", main_task1.TASK1_SPEED_PROFILE),
            ("task 2", main_task2.TASK2_SPEED_PROFILE),
        ):
            args = build_argument_parser(
                task_name=name,
                default_speed_profile=profile,
            ).parse_args([])
            self.assertEqual(
                (
                    args.ab_speed_cm_s,
                    args.bc_speed_cm_s,
                    args.cd_speed_cm_s,
                    args.da_speed_cm_s,
                ),
                (10.0, 10.0, 10.0, 10.0),
            )


if __name__ == "__main__":
    unittest.main()
