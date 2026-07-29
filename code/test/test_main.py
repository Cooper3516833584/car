"""Hardware-free tests for the one-lap radar entry point."""

from types import SimpleNamespace
import unittest

from main_fixed_track_test import (
    CompetitionCarApplication,
    MainConfig,
    RADAR_CENTER_BEHIND_A_ALONG_AB_CM,
    TRACK_SPEED_CM_S,
    build_argument_parser,
)


class CompetitionMainTests(unittest.TestCase):
    def test_defaults_are_for_one_low_speed_lap(self):
        config = MainConfig()
        self.assertEqual(config.radar_center_behind_a_cm, 0.0)
        self.assertEqual(config.speed_cm_s, 8.0)
        self.assertEqual(RADAR_CENTER_BEHIND_A_ALONG_AB_CM, 0.0)
        self.assertEqual(TRACK_SPEED_CM_S, 8.0)

    def test_cli_only_contains_radar_and_track_parameters(self):
        parser = build_argument_parser()
        args = parser.parse_args(
            [
                "--radar-port",
                "/dev/radar",
                "--radar-x-cm",
                "2.5",
                "--radar-y-cm",
                "-1.0",
                "--radar-yaw-cw-deg",
                "3.0",
                "--radar-center-behind-a-cm",
                "20.0",
                "--speed-cm-s",
                "6.0",
            ]
        )
        self.assertEqual(args.radar_port, "/dev/radar")
        self.assertEqual(
            (args.radar_x_cm, args.radar_y_cm, args.radar_yaw_cw_deg),
            (2.5, -1.0, 3.0),
        )
        self.assertEqual(args.radar_center_behind_a_cm, 20.0)
        self.assertEqual(args.speed_cm_s, 6.0)
        option_names = parser._option_string_actions
        self.assertNotIn("--task", option_names)
        self.assertNotIn("--link-port", option_names)

    def test_invalid_run_parameters_are_rejected(self):
        with self.assertRaises(ValueError):
            MainConfig(speed_cm_s=0.0)
        with self.assertRaises(ValueError):
            MainConfig(radar_center_behind_a_cm=-1.0)
        with self.assertRaises(ValueError):
            MainConfig(startup_scan_count=0)

    def test_completion_sets_application_event(self):
        app = CompetitionCarApplication(MainConfig())
        app.radar.set_motion_hint = lambda moving: setattr(
            app,
            "_last_motion_hint",
            moving,
        )
        app._on_follower_state(SimpleNamespace(completed=True))
        self.assertTrue(app._completed_event.is_set())
        self.assertFalse(app._last_motion_hint)

    def test_request_stop_only_sets_lifecycle_event(self):
        app = CompetitionCarApplication(MainConfig())
        self.assertFalse(app._stop_event.is_set())
        app.request_stop()
        self.assertTrue(app._stop_event.is_set())


if __name__ == "__main__":
    unittest.main()
