"""Hardware-free tests for the camera-only fixed-track entry point."""

import unittest

from main_camera_line_following import (
    CameraLineApplication,
    CameraLineMainConfig,
    RADAR_CENTER_BEHIND_A_ALONG_AB_CM,
    TRACK_SPEED_CM_S,
    build_argument_parser,
)


class CameraLineMainTests(unittest.TestCase):
    def test_editable_defaults_match_the_real_car_test_request(self):
        config = CameraLineMainConfig()
        self.assertEqual(TRACK_SPEED_CM_S, 40.0)
        self.assertEqual(RADAR_CENTER_BEHIND_A_ALONG_AB_CM, 0.0)
        self.assertEqual(config.speed_cm_s, 40.0)
        self.assertEqual(config.radar_center_behind_a_cm, 0.0)

    def test_camera_path_and_arguments_are_parsed_without_hardware(self):
        args = build_argument_parser().parse_args(
            ["--camera", "/dev/video2", "--speed-cm-s", "35", "--radar-center-behind-a-cm", "4"]
        )
        self.assertEqual(args.camera, "/dev/video2")
        self.assertEqual(args.speed_cm_s, 35.0)
        self.assertEqual(args.radar_center_behind_a_cm, 4.0)

    def test_control_speed_and_hardware_limit_follow_track_speed(self):
        config = CameraLineMainConfig(speed_cm_s=40.0)
        app = CameraLineApplication(config)
        control = app.follower.control_config
        self.assertEqual(control.cruise_speed_mm_s, 400.0)
        self.assertEqual(control.degraded_speed_mm_s, 220.0)
        self.assertEqual(control.short_loss_speed_mm_s, 140.0)
        self.assertEqual(control.minimum_tracking_speed_mm_s, 180.0)
        self.assertEqual(app.drive.rear_motors.max_wheel_speed_mm_s, 480.0)

    def test_invalid_values_are_rejected(self):
        with self.assertRaises(ValueError):
            CameraLineMainConfig(speed_cm_s=0.0)
        with self.assertRaises(ValueError):
            CameraLineMainConfig(speed_cm_s=101.0)
        with self.assertRaises(ValueError):
            CameraLineMainConfig(radar_center_behind_a_cm=-0.1)


if __name__ == "__main__":
    unittest.main()
