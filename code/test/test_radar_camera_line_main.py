"""Hardware-free tests for radar tracking with soft camera correction."""

import unittest
import time

from components.camera_line_correction import CameraLineCorrectionConfig
from components.camera_line_follower import LineObservation
from components.competition_track import TrackFollowerState, TrackSegment
from main_radar_camera_line_following import (
    CAMERA_CORRECTION_ENABLED,
    CAMERA_LATERAL_DEADBAND_CM,
    CAMERA_MAX_STEERING_CORRECTION_RAD,
    CAMERA_STEERING_GAIN_RAD_PER_CM,
    FINAL_DA_MIN_LEFT_CORRECTION_RAD,
    FINAL_DA_TRIM_FULL_PROGRESS_CM,
    FINAL_DA_TRIM_START_PROGRESS_CM,
    RADAR_CENTER_BEHIND_A_ALONG_AB_CM,
    TRACK_SPEED_CM_S,
    MainConfig,
    RadarCameraLineApplication,
    _CameraCorrectedDrive,
    build_argument_parser,
)


def observation(*, lateral_cm=0.0, round_marker=False):
    return LineObservation(
        timestamp_s=1.0,
        detected=True,
        confidence=0.95,
        lookahead_x_cm=35.0,
        lookahead_y_left_cm=lateral_cm,
        near_lateral_error_cm=lateral_cm,
        heading_error_rad=0.0,
        curvature_per_cm=0.0,
        fit_rmse_cm=0.5,
        visible_band_count=12,
        total_band_count=13,
        median_line_width_cm=28.0,
        polynomial_y_left_by_x=(0.0, 0.0, lateral_cm),
        dark_threshold=100.0,
        round_marker_detected=round_marker,
    )


class RadarCameraLineMainTests(unittest.TestCase):
    def test_editable_defaults_match_radar_fixed_track_entry(self):
        config = MainConfig()

        self.assertEqual(TRACK_SPEED_CM_S, 30.0)
        self.assertEqual(RADAR_CENTER_BEHIND_A_ALONG_AB_CM, 20.0)
        self.assertTrue(CAMERA_CORRECTION_ENABLED)
        self.assertEqual(CAMERA_LATERAL_DEADBAND_CM, 10.0)
        self.assertEqual(CAMERA_STEERING_GAIN_RAD_PER_CM, 0.010)
        self.assertEqual(CAMERA_MAX_STEERING_CORRECTION_RAD, 0.140)
        self.assertEqual(FINAL_DA_TRIM_START_PROGRESS_CM, 725.0)
        self.assertEqual(FINAL_DA_TRIM_FULL_PROGRESS_CM, 740.0)
        self.assertEqual(FINAL_DA_MIN_LEFT_CORRECTION_RAD, 0.100)
        self.assertEqual(config.speed_cm_s, 30.0)
        self.assertEqual(config.radar_center_behind_a_cm, 20.0)

    def test_final_da_trim_is_smooth_and_limited_to_lap_end(self):
        application = RadarCameraLineApplication(MainConfig())

        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.DA,
                progress_cm=724.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=-0.2,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        self.assertEqual(application._final_da_trim(), 0.0)

        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.DA,
                progress_cm=732.5,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=-0.2,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        self.assertAlmostEqual(application._final_da_trim(), 0.050)

        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.DA,
                progress_cm=750.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=-0.2,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        self.assertAlmostEqual(application._final_da_trim(), 0.100)

        application._on_follower_state(
            TrackFollowerState(
                running=False,
                completed=True,
                segment=TrackSegment.DA,
                progress_cm=773.0,
                target_speed_cm_s=0.0,
                commanded_speed_cm_s=0.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        self.assertEqual(application._final_da_trim(), 0.0)

    def test_fusion_config_preserves_fixed_track_inputs(self):
        config = MainConfig(
            radar_port="/dev/test-radar",
            startup_scan_count=5,
            calibration_timeout_s=12.0,
            radar_center_behind_a_cm=7.0,
            speed_cm_s=18.0,
        )

        self.assertEqual(config.radar_port, "/dev/test-radar")
        self.assertEqual(config.startup_scan_count, 5)
        self.assertEqual(config.calibration_timeout_s, 12.0)
        self.assertEqual(config.radar_center_behind_a_cm, 7.0)
        self.assertEqual(config.speed_cm_s, 18.0)
        self.assertEqual(RadarCameraLineApplication.__bases__, (object,))

    def test_camera_and_radar_arguments_are_parsed(self):
        args = build_argument_parser().parse_args(
            [
                "--camera",
                "/dev/video2",
                "--speed-cm-s",
                "35",
                "--radar-center-behind-a-cm",
                "4",
                "--no-camera-correction",
            ]
        )

        self.assertEqual(args.camera, "/dev/video2")
        self.assertEqual(args.speed_cm_s, 35.0)
        self.assertEqual(args.radar_center_behind_a_cm, 4.0)
        self.assertTrue(args.no_camera_correction)

    def test_current_front_camera_profile_is_retained(self):
        profile = RadarCameraLineApplication._front_camera_vision_config()

        self.assertEqual(profile.perspective.source_points_norm[0][1], 0.66)
        self.assertFalse(profile.require_adaptive_confirmation)
        self.assertTrue(profile.use_expected_width_window)
        self.assertEqual(profile.scan_far_cm, 72.0)
        self.assertEqual(profile.expected_line_width_cm, 28.0)
        self.assertEqual(profile.maximum_center_jump_cm, 18.0)

    def test_large_visual_error_only_adds_small_correction(self):
        correction = CameraLineCorrectionConfig(
            required_consecutive_frames=1,
            large_error_required_frames=1,
            lateral_deadband_cm=10.0,
            steering_gain_rad_per_cm=0.006,
            maximum_abs_correction_rad=0.055,
            correction_filter_time_constant_s=0.0,
            maximum_correction_rate_rad_s=100.0,
        )
        app = RadarCameraLineApplication(
            MainConfig(camera_correction=correction)
        )
        now = time.monotonic()
        app.camera_corrector.update_from_observation(
            observation(lateral_cm=15.0),
            now_s=now,
        )

        adjusted = app._adjust_radar_steering(
            0.10,
            20.0,
        )

        self.assertAlmostEqual(adjusted, 0.13)
        self.assertLess(
            adjusted - 0.10,
            CAMERA_MAX_STEERING_CORRECTION_RAD,
        )

    def test_fusion_drive_applies_correction_without_shared_follower_changes(self):
        class FakeDrive:
            def __init__(self):
                self.motion = None

            def set_motion(self, speed, steering, *args, **kwargs):
                self.motion = (speed, steering, args, kwargs)
                return "plan"

            def stop(self, *args, **kwargs):
                return (args, kwargs)

        drive = FakeDrive()
        fused = _CameraCorrectedDrive(
            drive,
            lambda steering, speed_cm_s: steering
            + speed_cm_s / 1000.0,
        )

        result = fused.set_motion(
            300.0,
            0.10,
            rear_differential_linked=True,
        )

        self.assertEqual(result, "plan")
        self.assertEqual(drive.motion[0], 300.0)
        self.assertAlmostEqual(drive.motion[1], 0.13)
        self.assertTrue(drive.motion[3]["rear_differential_linked"])

    def test_marker_never_creates_a_new_visual_correction(self):
        correction = CameraLineCorrectionConfig(
            required_consecutive_frames=1,
            large_error_required_frames=1,
            correction_filter_time_constant_s=0.0,
            maximum_correction_rate_rad_s=100.0,
        )
        app = RadarCameraLineApplication(
            MainConfig(camera_correction=correction)
        )
        now = time.monotonic()
        state = app.camera_corrector.update_from_observation(
            observation(lateral_cm=30.0, round_marker=True),
            now_s=now,
        )

        adjusted = app._adjust_radar_steering(-0.08, 20.0)

        self.assertFalse(state.active)
        self.assertEqual(adjusted, -0.08)

    def test_disabled_camera_returns_unmodified_radar_steering(self):
        app = RadarCameraLineApplication(
            MainConfig(camera_correction_enabled=False)
        )

        self.assertEqual(
            app._adjust_radar_steering(0.123, 30.0),
            0.123,
        )

    def test_follower_segment_selects_curve_recovery_mode(self):
        app = RadarCameraLineApplication(MainConfig())

        app._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.BC,
                progress_cm=150.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=-0.1,
                cross_track_error_cm=2.0,
                heading_error_deg=-3.0,
            )
        )
        self.assertTrue(app.camera_corrector.state.curve_mode)

        app._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.CD,
                progress_cm=390.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=1.0,
                heading_error_deg=0.0,
            )
        )
        self.assertFalse(app.camera_corrector.state.curve_mode)

    def test_invalid_values_are_rejected(self):
        with self.assertRaises(ValueError):
            MainConfig(speed_cm_s=0.0)
        with self.assertRaises(ValueError):
            MainConfig(radar_center_behind_a_cm=-0.1)
        with self.assertRaises(ValueError):
            MainConfig(camera_source=-1)


if __name__ == "__main__":
    unittest.main()
