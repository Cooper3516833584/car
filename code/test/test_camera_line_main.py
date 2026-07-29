"""Hardware-free tests for the camera-only fixed-track entry point."""

import unittest

from main_camera_line_following import (
    CameraLineApplication,
    CameraLineMainConfig,
    RADAR_CENTER_BEHIND_A_ALONG_AB_CM,
    TRACK_AB_SPEED_CM_S,
    TRACK_BC_SPEED_CM_S,
    TRACK_CD_SPEED_CM_S,
    TRACK_DA_SPEED_CM_S,
    TRACK_SPEED_CM_S,
    build_argument_parser,
)


class CameraLineMainTests(unittest.TestCase):
    def test_editable_defaults_match_the_real_car_test_request(self):
        config = CameraLineMainConfig()
        self.assertEqual(TRACK_SPEED_CM_S, 40.0)
        self.assertEqual(
            (
                TRACK_AB_SPEED_CM_S,
                TRACK_BC_SPEED_CM_S,
                TRACK_CD_SPEED_CM_S,
                TRACK_DA_SPEED_CM_S,
            ),
            (40.0, 40.0, 40.0, 40.0),
        )
        self.assertEqual(RADAR_CENTER_BEHIND_A_ALONG_AB_CM, 0.0)
        self.assertEqual(config.segment_speeds_cm_s, (40.0, 40.0, 40.0, 40.0))
        self.assertEqual(config.radar_center_behind_a_cm, 0.0)

    def test_camera_path_and_arguments_are_parsed_without_hardware(self):
        args = build_argument_parser().parse_args(
            ["--camera", "/dev/video2", "--speed-cm-s", "35", "--radar-center-behind-a-cm", "4"]
        )
        self.assertEqual(args.camera, "/dev/video2")
        self.assertEqual(args.speed_cm_s, 35.0)
        self.assertEqual(args.radar_center_behind_a_cm, 4.0)

    def test_control_speed_and_hardware_limit_follow_track_speed(self):
        config = CameraLineMainConfig(uniform_speed_cm_s=40.0)
        app = CameraLineApplication(config)
        control = app.follower.control_config
        self.assertEqual(control.cruise_speed_mm_s, 400.0)
        self.assertEqual(control.degraded_speed_mm_s, 220.0)
        self.assertEqual(control.short_loss_speed_mm_s, 140.0)
        self.assertEqual(control.minimum_tracking_speed_mm_s, 180.0)
        self.assertEqual(app.drive.rear_motors.max_wheel_speed_mm_s, 480.0)

    def test_current_front_camera_profile_excludes_the_visible_car_body(self):
        profile = CameraLineApplication._front_camera_vision_config()
        self.assertEqual(profile.perspective.source_points_norm[0][1], 0.66)
        self.assertFalse(profile.require_adaptive_confirmation)
        self.assertTrue(profile.use_expected_width_window)
        self.assertEqual(profile.scan_far_cm, 72.0)
        self.assertEqual(profile.expected_line_width_cm, 28.0)
        self.assertEqual(profile.maximum_line_internal_gap_cm, 8.0)

    def test_round_markers_select_the_four_independent_segment_speeds(self):
        app = CameraLineApplication(
            CameraLineMainConfig(
                ab_speed_cm_s=10.0,
                bc_speed_cm_s=11.0,
                cd_speed_cm_s=12.0,
                da_speed_cm_s=13.0,
            )
        )
        self.assertEqual(app.follower._active_cruise_speed_mm_s, 100.0)
        app._on_marker_passed(1)
        self.assertEqual(app.follower._active_cruise_speed_mm_s, 110.0)
        app._on_marker_passed(2)
        self.assertEqual(app.follower._active_cruise_speed_mm_s, 120.0)
        app._on_marker_passed(3)
        self.assertEqual(app.follower._active_cruise_speed_mm_s, 130.0)

    def test_invalid_values_are_rejected(self):
        with self.assertRaises(ValueError):
            CameraLineMainConfig(ab_speed_cm_s=0.0)
        with self.assertRaises(ValueError):
            CameraLineMainConfig(bc_speed_cm_s=101.0)
        with self.assertRaises(ValueError):
            CameraLineMainConfig(radar_center_behind_a_cm=-0.1)


if __name__ == "__main__":
    unittest.main()
