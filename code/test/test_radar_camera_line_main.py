"""Hardware-free tests for radar tracking with soft camera correction."""

import math
import unittest
import time

from components.camera_line_correction import (
    CameraLineCorrectionConfig,
    CameraLineCorrectionState,
)
from components.camera_line_follower import LineObservation
from components.competition_track import TrackFollowerState, TrackSegment
from components.fleet_models import (
    AckReason,
    AckStatus,
    NodeFlags,
)
from components.navigation import NavigationPose
from main_radar_camera_line_following import (
    AB_LINE_ASSIST_FADE_END_PROGRESS_CM,
    AB_LINE_ASSIST_FULL_END_PROGRESS_CM,
    AB_LINE_ASSIST_GAIN_RAD_PER_CM,
    AB_LINE_ASSIST_LATERAL_DEADBAND_CM,
    AB_LINE_ASSIST_MAX_CORRECTION_RAD,
    AB_LINE_ASSIST_MIN_VALID_FRAMES,
    AB_TRACK_SPEED_CM_S,
    AB_START_ALIGNMENT_FADE_END_PROGRESS_CM,
    AB_START_ALIGNMENT_FULL_END_PROGRESS_CM,
    AB_START_HEADING_GAIN,
    AB_START_MAX_HEADING_CORRECTION_RAD,
    AB_START_MAX_TOTAL_CAMERA_CORRECTION_RAD,
    AB_START_MIN_VALID_FRAMES,
    AB_START_MAX_CURVATURE_PER_CM,
    AB_START_MAX_FORWARD_HEADING_CHANGE_RAD,
    BC_ENTRY_LIMIT_END_PROGRESS_CM,
    BC_ENTRY_MIN_RIGHT_CORRECTION_RAD,
    C_VISIBLE_MIN_LEFT_CORRECTION_RAD,
    C_VISIBLE_TRIM_END_PROGRESS_CM,
    C_VISIBLE_TRIM_FADE_START_PROGRESS_CM,
    C_VISIBLE_TRIM_FULL_PROGRESS_CM,
    C_VISIBLE_TRIM_START_PROGRESS_CM,
    CAMERA_CORRECTION_ENABLED,
    CAMERA_LATERAL_DEADBAND_CM,
    CAMERA_MAX_STEERING_CORRECTION_RAD,
    CAMERA_STEERING_GAIN_RAD_PER_CM,
    CAR_OPERATION_LOCALIZATION_LOST,
    BC_TRACK_SPEED_CM_S,
    CD_TRACK_SPEED_CM_S,
    DA_TRACK_SPEED_CM_S,
    DA_VISIBLE_EXTRA_LEFT_CORRECTION_RAD,
    DA_VISIBLE_MIN_LEFT_CORRECTION_RAD,
    DA_VISIBLE_TRIM_FULL_PROGRESS_CM,
    DA_VISIBLE_TRIM_START_PROGRESS_CM,
    FINAL_DA_MIN_LEFT_CORRECTION_RAD,
    FINAL_DA_MAX_TOTAL_LEFT_CORRECTION_RAD,
    FINAL_DA_TRIM_FULL_PROGRESS_CM,
    FINAL_DA_TRIM_START_PROGRESS_CM,
    FINAL_DA_VISUAL_DEADBAND_CM,
    FINAL_DA_VISUAL_GAIN_RAD_PER_CM,
    FINAL_DA_VISUAL_HOLD_S,
    FINAL_DA_VISUAL_MAX_RESIDUAL_RAD,
    FINAL_A_MAX_CAMERA_ERROR_CM,
    FLEET_TERMINAL_REPORT_GRACE_S,
    FLEET_POSITION_REPORTING_ENABLED,
    RADAR_CENTER_BEHIND_A_ALONG_AB_CM,
    MainConfig,
    RadarCameraLineApplication,
    _CameraCorrectedDrive,
    build_argument_parser,
)


def observation(
    *,
    lateral_cm=0.0,
    heading_error_rad=0.0,
    curvature_per_cm=0.0,
    forward_heading_change_rad=0.0,
    round_marker=False,
    transverse=False,
):
    return LineObservation(
        timestamp_s=1.0,
        detected=True,
        confidence=0.95,
        lookahead_x_cm=35.0,
        lookahead_y_left_cm=lateral_cm,
        near_lateral_error_cm=lateral_cm,
        heading_error_rad=heading_error_rad,
        curvature_per_cm=curvature_per_cm,
        fit_rmse_cm=0.5,
        visible_band_count=12,
        total_band_count=13,
        median_line_width_cm=28.0,
        polynomial_y_left_by_x=(0.0, 0.0, lateral_cm),
        dark_threshold=100.0,
        round_marker_detected=round_marker,
        transverse_line_detected=transverse,
        forward_heading_change_rad=forward_heading_change_rad,
    )


class RadarCameraLineMainTests(unittest.TestCase):
    def test_editable_defaults_match_radar_fixed_track_entry(self):
        config = MainConfig()

        self.assertEqual(AB_TRACK_SPEED_CM_S, 30.0)
        self.assertEqual(BC_TRACK_SPEED_CM_S, 30.0)
        self.assertEqual(CD_TRACK_SPEED_CM_S, 30.0)
        self.assertEqual(DA_TRACK_SPEED_CM_S, 30.0)
        self.assertTrue(FLEET_POSITION_REPORTING_ENABLED)
        self.assertEqual(RADAR_CENTER_BEHIND_A_ALONG_AB_CM, 18.625)
        self.assertTrue(CAMERA_CORRECTION_ENABLED)
        self.assertEqual(CAMERA_LATERAL_DEADBAND_CM, 10.0)
        self.assertEqual(CAMERA_STEERING_GAIN_RAD_PER_CM, 0.010)
        self.assertEqual(CAMERA_MAX_STEERING_CORRECTION_RAD, 0.140)
        self.assertEqual(AB_START_ALIGNMENT_FULL_END_PROGRESS_CM, 80.0)
        self.assertEqual(AB_START_ALIGNMENT_FADE_END_PROGRESS_CM, 100.0)
        self.assertEqual(AB_START_HEADING_GAIN, 1.30)
        self.assertEqual(AB_START_MAX_HEADING_CORRECTION_RAD, 0.180)
        self.assertEqual(
            AB_START_MAX_TOTAL_CAMERA_CORRECTION_RAD,
            0.220,
        )
        self.assertEqual(AB_START_MIN_VALID_FRAMES, 2)
        self.assertEqual(AB_START_MAX_CURVATURE_PER_CM, 0.003)
        self.assertEqual(
            AB_START_MAX_FORWARD_HEADING_CHANGE_RAD,
            0.080,
        )
        self.assertEqual(AB_LINE_ASSIST_FULL_END_PROGRESS_CM, 100.0)
        self.assertEqual(AB_LINE_ASSIST_FADE_END_PROGRESS_CM, 135.0)
        self.assertEqual(AB_LINE_ASSIST_LATERAL_DEADBAND_CM, 2.0)
        self.assertEqual(AB_LINE_ASSIST_GAIN_RAD_PER_CM, 0.005)
        self.assertEqual(AB_LINE_ASSIST_MAX_CORRECTION_RAD, 0.060)
        self.assertEqual(AB_LINE_ASSIST_MIN_VALID_FRAMES, 3)
        self.assertEqual(BC_ENTRY_LIMIT_END_PROGRESS_CM, 210.0)
        self.assertEqual(BC_ENTRY_MIN_RIGHT_CORRECTION_RAD, -0.012)
        self.assertEqual(C_VISIBLE_TRIM_START_PROGRESS_CM, 300.0)
        self.assertEqual(C_VISIBLE_TRIM_FULL_PROGRESS_CM, 330.0)
        self.assertEqual(C_VISIBLE_TRIM_FADE_START_PROGRESS_CM, 390.0)
        self.assertEqual(C_VISIBLE_TRIM_END_PROGRESS_CM, 430.0)
        self.assertEqual(C_VISIBLE_MIN_LEFT_CORRECTION_RAD, 0.035)
        self.assertEqual(DA_VISIBLE_TRIM_START_PROGRESS_CM, 560.0)
        self.assertEqual(DA_VISIBLE_TRIM_FULL_PROGRESS_CM, 590.0)
        self.assertEqual(DA_VISIBLE_MIN_LEFT_CORRECTION_RAD, 0.045)
        self.assertEqual(DA_VISIBLE_EXTRA_LEFT_CORRECTION_RAD, 0.030)
        self.assertEqual(FINAL_DA_TRIM_START_PROGRESS_CM, 725.0)
        self.assertEqual(FINAL_DA_TRIM_FULL_PROGRESS_CM, 740.0)
        self.assertEqual(FINAL_DA_MIN_LEFT_CORRECTION_RAD, 0.100)
        self.assertEqual(FINAL_DA_VISUAL_DEADBAND_CM, 3.0)
        self.assertEqual(FINAL_DA_VISUAL_GAIN_RAD_PER_CM, 0.005)
        self.assertEqual(FINAL_DA_VISUAL_MAX_RESIDUAL_RAD, 0.040)
        self.assertEqual(FINAL_DA_VISUAL_HOLD_S, 0.65)
        self.assertEqual(FINAL_DA_MAX_TOTAL_LEFT_CORRECTION_RAD, 0.170)
        self.assertEqual(FINAL_A_MAX_CAMERA_ERROR_CM, 6.0)
        self.assertEqual(CAR_OPERATION_LOCALIZATION_LOST, 10)
        self.assertEqual(FLEET_TERMINAL_REPORT_GRACE_S, 3.0)
        self.assertEqual(
            (
                config.speed_profile.ab_cm_s,
                config.speed_profile.bc_cm_s,
                config.speed_profile.cd_cm_s,
                config.speed_profile.da_cm_s,
            ),
            (30.0, 30.0, 30.0, 30.0),
        )
        self.assertEqual(config.radar_center_behind_a_cm, 18.625)

    def test_track_extends_one_lap_until_rear_axle_reaches_a(self):
        config = MainConfig()
        application = RadarCameraLineApplication(config)
        one_lap_progress_cm = application.track.point_at_index(
            application.track.wrap_start_index
        ).progress_cm

        self.assertAlmostEqual(
            application.track.finish_progress_cm - one_lap_progress_cm,
            config.radar_center_behind_a_cm,
        )
        finish_point = next(
            point
            for point in application.track.points
            if math.isclose(
                point.progress_cm,
                application.track.finish_progress_cm,
                abs_tol=1e-9,
            )
        )
        self.assertAlmostEqual(finish_point.x_cm, config.radar_center_behind_a_cm)
        self.assertAlmostEqual(finish_point.y_cm, 0.0)
        self.assertGreater(
            application.track.points[-1].progress_cm,
            application.track.finish_progress_cm,
        )
        self.assertEqual(
            application.track.segment_at_progress(
                application.track.finish_progress_cm
            ),
            TrackSegment.AB,
        )

    def test_final_da_trim_is_smooth_and_continues_to_terminal_a(self):
        application = RadarCameraLineApplication(MainConfig())

        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.DA,
                progress_cm=559.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=-0.2,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        self.assertIsNone(application._final_da_trim())

        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.DA,
                progress_cm=575.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=-0.2,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        self.assertAlmostEqual(application._final_da_trim(), 0.0225)

        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.DA,
                progress_cm=650.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=-0.2,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        self.assertAlmostEqual(application._final_da_trim(), 0.045)

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
        self.assertAlmostEqual(application._final_da_trim(), 0.0725)

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
                running=True,
                completed=False,
                segment=TrackSegment.AB,
                progress_cm=application.track.finish_progress_cm - 2.0,
                target_speed_cm_s=8.0,
                commanded_speed_cm_s=8.0,
                steering_angle_rad=-0.1,
                cross_track_error_cm=2.0,
                heading_error_deg=-3.0,
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
        self.assertIsNone(application._final_da_trim())

    def test_c_point_trim_is_limited_to_visible_offset_region(self):
        application = RadarCameraLineApplication(MainConfig())

        cases = (
            (TrackSegment.BC, 299.0, None),
            (TrackSegment.BC, 315.0, 0.0175),
            (TrackSegment.BC, 350.0, 0.035),
            (TrackSegment.CD, 390.0, 0.035),
            (TrackSegment.CD, 410.0, 0.0175),
            (TrackSegment.CD, 430.0, None),
        )
        for segment, progress_cm, expected in cases:
            application._on_follower_state(
                TrackFollowerState(
                    running=True,
                    completed=False,
                    segment=segment,
                    progress_cm=progress_cm,
                    target_speed_cm_s=15.0,
                    commanded_speed_cm_s=15.0,
                    steering_angle_rad=0.0,
                    cross_track_error_cm=0.0,
                    heading_error_deg=0.0,
                )
            )
            actual = application._c_point_trim()
            if expected is None:
                self.assertIsNone(actual)
            else:
                self.assertAlmostEqual(actual, expected)

    def test_da_extra_trim_overcomes_an_already_active_camera_floor(self):
        application = RadarCameraLineApplication(MainConfig())

        cases = (
            (559.0, None),
            (575.0, 0.015),
            (650.0, 0.030),
            (732.5, 0.015),
            (740.0, None),
        )
        for progress_cm, expected in cases:
            application._on_follower_state(
                TrackFollowerState(
                    running=True,
                    completed=False,
                    segment=TrackSegment.DA,
                    progress_cm=progress_cm,
                    target_speed_cm_s=15.0,
                    commanded_speed_cm_s=15.0,
                    steering_angle_rad=-0.2,
                    cross_track_error_cm=5.0,
                    heading_error_deg=-18.0,
                )
            )
            actual = application._da_visible_extra_trim()
            if expected is None:
                self.assertIsNone(actual)
            else:
                self.assertAlmostEqual(actual, expected)

    def test_final_da_adds_visual_residual_to_fixed_feed_forward(self):
        application = RadarCameraLineApplication(MainConfig())
        now = time.monotonic()
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.DA,
                progress_cm=750.0,
                target_speed_cm_s=15.0,
                commanded_speed_cm_s=15.0,
                steering_angle_rad=-0.32,
                cross_track_error_cm=4.0,
                heading_error_deg=-18.0,
            )
        )
        application._on_camera_state(
            CameraLineCorrectionState(
                running=True,
                active=True,
                timestamp_s=now,
                confidence=0.82,
                lateral_error_cm=15.0,
                valid_frames=4,
                curve_mode=True,
                observation=observation(lateral_cm=15.0),
            )
        )

        self.assertAlmostEqual(
            application._final_da_visual_residual(now_s=now),
            0.040,
        )
        self.assertAlmostEqual(
            application._adjust_radar_steering(-0.32, 15.0),
            -0.18,
        )

    def test_final_da_precise_visual_error_preserves_known_good_trim(self):
        application = RadarCameraLineApplication(MainConfig())
        now = time.monotonic()
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.DA,
                progress_cm=750.0,
                target_speed_cm_s=15.0,
                commanded_speed_cm_s=15.0,
                steering_angle_rad=-0.32,
                cross_track_error_cm=5.0,
                heading_error_deg=-18.0,
            )
        )
        application._on_camera_state(
            CameraLineCorrectionState(
                running=True,
                active=True,
                timestamp_s=now,
                confidence=0.82,
                lateral_error_cm=2.0,
                valid_frames=4,
                curve_mode=True,
                observation=observation(lateral_cm=2.0),
            )
        )

        self.assertEqual(
            application._final_da_visual_residual(now_s=now),
            0.0,
        )
        self.assertAlmostEqual(
            application._adjust_radar_steering(-0.32, 15.0),
            -0.22,
        )

    def test_final_da_holds_last_trusted_feedback_through_marker(self):
        application = RadarCameraLineApplication(MainConfig())
        now = time.monotonic()
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.DA,
                progress_cm=750.0,
                target_speed_cm_s=15.0,
                commanded_speed_cm_s=15.0,
                steering_angle_rad=-0.32,
                cross_track_error_cm=5.0,
                heading_error_deg=-18.0,
            )
        )
        application._on_camera_state(
            CameraLineCorrectionState(
                running=True,
                active=True,
                timestamp_s=now,
                confidence=0.82,
                lateral_error_cm=12.0,
                valid_frames=4,
                curve_mode=True,
                observation=observation(lateral_cm=12.0),
            )
        )
        application._on_camera_state(
            CameraLineCorrectionState(
                running=True,
                active=False,
                timestamp_s=now + 0.1,
                confidence=0.30,
                curve_mode=True,
                observation=observation(
                    lateral_cm=14.0,
                    transverse=True,
                ),
            )
        )

        self.assertAlmostEqual(
            application._final_da_visual_residual(now_s=now + 0.5),
            0.040,
        )
        self.assertEqual(
            application._final_da_visual_residual(now_s=now + 0.7),
            0.0,
        )

    def test_post_lap_extension_refreshes_terminal_visual_feedback(self):
        application = RadarCameraLineApplication(MainConfig())
        now = time.monotonic()
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.AB,
                progress_cm=application.track.finish_progress_cm - 2.0,
                target_speed_cm_s=12.0,
                commanded_speed_cm_s=12.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=2.0,
                heading_error_deg=1.0,
            )
        )
        application._on_camera_state(
            CameraLineCorrectionState(
                running=True,
                active=True,
                timestamp_s=now,
                confidence=0.82,
                lateral_error_cm=8.0,
                valid_frames=4,
                curve_mode=False,
                observation=observation(lateral_cm=8.0),
            )
        )

        self.assertEqual(application._final_da_visual_error_cm, 8.0)
        self.assertEqual(application._final_da_visual_timestamp_s, now)
        self.assertEqual(
            application._final_da_visual_residual(now_s=now),
            0.025,
        )
        self.assertAlmostEqual(
            application._adjust_radar_steering(-0.20, 8.0),
            -0.075,
        )

    def test_terminal_camera_disagreement_degrades_reported_pose_quality(self):
        application = RadarCameraLineApplication(MainConfig())
        now = time.monotonic()
        with application._lock:
            application._ready = True
            application._latest_navigation_pose = NavigationPose(
                0.0,
                0.0,
                0.0,
                now,
            )
            application._final_da_visual_error_cm = 12.0
            application._final_da_visual_timestamp_s = now

        application._on_follower_state(
            TrackFollowerState(
                running=False,
                completed=True,
                segment=TrackSegment.AB,
                progress_cm=772.0,
                target_speed_cm_s=0.0,
                commanded_speed_cm_s=0.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=4.0,
                heading_error_deg=-18.0,
            )
        )

        state = application._fleet_state()
        self.assertTrue(
            state.node_flags & int(NodeFlags.LOCALIZATION_DEGRADED)
        )
        self.assertFalse(state.node_flags & int(NodeFlags.POSE_VALID))
        self.assertEqual(state.pose_quality, 2)
        self.assertEqual(
            state.operation_state,
            CAR_OPERATION_LOCALIZATION_LOST,
        )

    def test_accurate_terminal_camera_error_still_reports_arrived(self):
        application = RadarCameraLineApplication(MainConfig())
        now = time.monotonic()
        with application._lock:
            application._ready = True
            application._latest_navigation_pose = NavigationPose(
                0.0,
                0.0,
                0.0,
                now,
            )
            application._final_da_visual_error_cm = 2.0
            application._final_da_visual_timestamp_s = now

        application._on_follower_state(
            TrackFollowerState(
                running=False,
                completed=True,
                segment=TrackSegment.AB,
                progress_cm=772.0,
                target_speed_cm_s=0.0,
                commanded_speed_cm_s=0.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=5.0,
                heading_error_deg=-18.0,
            )
        )

        state = application._fleet_state()
        self.assertTrue(state.node_flags & int(NodeFlags.POSE_VALID))
        self.assertFalse(
            state.node_flags & int(NodeFlags.LOCALIZATION_DEGRADED)
        )
        self.assertEqual(state.pose_quality, 4)
        self.assertEqual(state.operation_state, 7)

    def test_course_trims_are_applied_by_steering_fusion(self):
        application = RadarCameraLineApplication(MainConfig())
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.BC,
                progress_cm=350.0,
                target_speed_cm_s=15.0,
                commanded_speed_cm_s=15.0,
                steering_angle_rad=-0.18,
                cross_track_error_cm=8.0,
                heading_error_deg=-19.0,
            )
        )
        self.assertAlmostEqual(
            application._adjust_radar_steering(-0.18, 15.0),
            -0.145,
        )

        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.DA,
                progress_cm=650.0,
                target_speed_cm_s=15.0,
                commanded_speed_cm_s=15.0,
                steering_angle_rad=-0.20,
                cross_track_error_cm=5.0,
                heading_error_deg=-19.0,
            )
        )
        self.assertAlmostEqual(
            application._adjust_radar_steering(-0.20, 15.0),
            -0.125,
        )

    def test_inactive_final_trim_does_not_clamp_negative_camera_correction(self):
        correction = CameraLineCorrectionConfig(
            required_consecutive_frames=1,
            large_error_required_frames=1,
            lateral_deadband_cm=10.0,
            steering_gain_rad_per_cm=0.010,
            maximum_abs_correction_rad=0.140,
            correction_filter_time_constant_s=0.0,
            maximum_correction_rate_rad_s=100.0,
        )
        application = RadarCameraLineApplication(
            MainConfig(camera_correction=correction)
        )
        application.camera_corrector.update_from_observation(
            observation(lateral_cm=-25.0),
            now_s=time.monotonic(),
        )

        adjusted = application._adjust_radar_steering(0.020, 20.0)

        self.assertLess(adjusted, 0.020)
        self.assertAlmostEqual(adjusted, -0.120)

    def test_first_curve_entry_limits_ambiguous_right_camera_correction(self):
        application = RadarCameraLineApplication(MainConfig())
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.BC,
                progress_cm=180.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=-0.15,
                cross_track_error_cm=4.0,
                heading_error_deg=-10.0,
            )
        )

        self.assertEqual(
            application._apply_course_camera_limit(-0.080),
            -0.012,
        )
        self.assertEqual(
            application._apply_course_camera_limit(0.080),
            0.080,
        )

    def test_ab_start_uses_strong_camera_heading_alignment(self):
        correction = CameraLineCorrectionConfig(
            required_consecutive_frames=2,
            large_error_required_frames=2,
            correction_filter_time_constant_s=0.0,
            maximum_correction_rate_rad_s=100.0,
        )
        application = RadarCameraLineApplication(
            MainConfig(camera_correction=correction)
        )
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.AB,
                progress_cm=20.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        for index in range(2):
            application.camera_corrector.update_from_observation(
                observation(heading_error_rad=0.10),
                now_s=1.0 + 0.06 * index,
            )

        self.assertAlmostEqual(
            application._ab_start_alignment_correction(now_s=1.06),
            0.130,
        )

    def test_ab_start_alignment_fades_before_first_curve(self):
        correction = CameraLineCorrectionConfig(
            required_consecutive_frames=2,
            large_error_required_frames=2,
            correction_filter_time_constant_s=0.0,
            maximum_correction_rate_rad_s=100.0,
        )
        application = RadarCameraLineApplication(
            MainConfig(camera_correction=correction)
        )
        for index in range(2):
            application.camera_corrector.update_from_observation(
                observation(heading_error_rad=-0.10),
                now_s=1.0 + 0.06 * index,
            )
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.AB,
                progress_cm=90.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )

        self.assertAlmostEqual(
            application._ab_start_alignment_correction(now_s=1.06),
            -0.065,
        )

    def test_ab_line_assist_fills_small_lateral_deadband(self):
        correction = CameraLineCorrectionConfig(
            required_consecutive_frames=2,
            large_error_required_frames=2,
            correction_filter_time_constant_s=0.0,
            maximum_correction_rate_rad_s=100.0,
        )
        application = RadarCameraLineApplication(
            MainConfig(camera_correction=correction)
        )
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.AB,
                progress_cm=75.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        for index in range(3):
            timestamp_s = 1.0 + 0.06 * index
            application.camera_corrector.update_from_observation(
                observation(lateral_cm=7.0),
                now_s=timestamp_s,
            )

        self.assertAlmostEqual(
            application._ab_line_assist_correction(now_s=1.12),
            0.025,
        )

    def test_ab_line_assist_fades_before_b_without_persisting(self):
        correction = CameraLineCorrectionConfig(
            required_consecutive_frames=2,
            large_error_required_frames=2,
            correction_filter_time_constant_s=0.0,
            maximum_correction_rate_rad_s=100.0,
        )
        application = RadarCameraLineApplication(
            MainConfig(camera_correction=correction)
        )
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.AB,
                progress_cm=117.5,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        for index in range(3):
            timestamp_s = 1.0 + 0.06 * index
            application.camera_corrector.update_from_observation(
                observation(lateral_cm=20.0),
                now_s=timestamp_s,
            )

        self.assertAlmostEqual(
            application._ab_line_assist_correction(now_s=1.12),
            0.030,
        )

        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.BC,
                progress_cm=160.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=-0.1,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        self.assertIsNone(
            application._ab_line_assist_correction(now_s=1.12)
        )

    def test_ab_visual_assists_reject_non_straight_camera_fit(self):
        correction = CameraLineCorrectionConfig(
            required_consecutive_frames=2,
            large_error_required_frames=2,
            correction_filter_time_constant_s=0.0,
            maximum_correction_rate_rad_s=100.0,
        )
        application = RadarCameraLineApplication(
            MainConfig(camera_correction=correction)
        )
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.AB,
                progress_cm=60.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        curved = observation(
            lateral_cm=20.0,
            heading_error_rad=math.radians(4.0),
            curvature_per_cm=0.01,
            forward_heading_change_rad=0.20,
        )
        for index in range(3):
            application.camera_corrector.update_from_observation(
                curved,
                now_s=1.0 + 0.06 * index,
            )

        self.assertIsNone(
            application._ab_line_assist_correction(now_s=1.12)
        )
        self.assertEqual(
            application._ab_start_alignment_correction(now_s=1.12),
            0.0,
        )

    def test_ab_start_alignment_rejects_start_line_and_marker(self):
        correction = CameraLineCorrectionConfig(
            required_consecutive_frames=2,
            large_error_required_frames=2,
            correction_filter_time_constant_s=0.0,
            maximum_correction_rate_rad_s=100.0,
        )
        for rejected in (
            observation(heading_error_rad=0.20, round_marker=True),
            observation(heading_error_rad=0.20, transverse=True),
        ):
            application = RadarCameraLineApplication(
                MainConfig(camera_correction=correction)
            )
            application._on_follower_state(
                TrackFollowerState(
                    running=True,
                    completed=False,
                    segment=TrackSegment.AB,
                    progress_cm=10.0,
                    target_speed_cm_s=30.0,
                    commanded_speed_cm_s=30.0,
                    steering_angle_rad=0.0,
                    cross_track_error_cm=0.0,
                    heading_error_deg=0.0,
                )
            )
            for index in range(2):
                application.camera_corrector.update_from_observation(
                    rejected,
                    now_s=1.0 + 0.06 * index,
                )

            self.assertEqual(
                application._ab_start_alignment_correction(),
                0.0,
            )

    def test_first_curve_limit_does_not_affect_straights_or_late_curve(self):
        application = RadarCameraLineApplication(MainConfig())
        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.AB,
                progress_cm=100.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )
        )
        self.assertEqual(
            application._apply_course_camera_limit(-0.080),
            -0.080,
        )

        application._on_follower_state(
            TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.BC,
                progress_cm=220.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=-0.1,
                cross_track_error_cm=4.0,
                heading_error_deg=-10.0,
            )
        )
        self.assertEqual(
            application._apply_course_camera_limit(-0.080),
            -0.080,
        )

    def test_fusion_config_preserves_fixed_track_inputs(self):
        config = MainConfig(
            radar_port="/dev/test-radar",
            startup_scan_count=5,
            calibration_timeout_s=12.0,
            radar_center_behind_a_cm=7.0,
            ab_speed_cm_s=18.0,
            bc_speed_cm_s=19.0,
            cd_speed_cm_s=20.0,
            da_speed_cm_s=21.0,
        )

        self.assertEqual(config.radar_port, "/dev/test-radar")
        self.assertEqual(config.startup_scan_count, 5)
        self.assertEqual(config.calibration_timeout_s, 12.0)
        self.assertEqual(config.radar_center_behind_a_cm, 7.0)
        self.assertEqual(
            (
                config.speed_profile.ab_cm_s,
                config.speed_profile.bc_cm_s,
                config.speed_profile.cd_cm_s,
                config.speed_profile.da_cm_s,
            ),
            (18.0, 19.0, 20.0, 21.0),
        )
        self.assertEqual(RadarCameraLineApplication.__bases__, (object,))

    def test_camera_and_radar_arguments_are_parsed(self):
        args = build_argument_parser().parse_args(
            [
                "--camera",
                "/dev/video2",
                "--ab-speed-cm-s",
                "31",
                "--bc-speed-cm-s",
                "32",
                "--cd-speed-cm-s",
                "33",
                "--da-speed-cm-s",
                "34",
                "--radar-center-behind-a-cm",
                "4",
                "--no-camera-correction",
                "--fleet-position-only",
            ]
        )

        self.assertEqual(args.camera, "/dev/video2")
        self.assertEqual(
            (
                args.ab_speed_cm_s,
                args.bc_speed_cm_s,
                args.cd_speed_cm_s,
                args.da_speed_cm_s,
            ),
            (31.0, 32.0, 33.0, 34.0),
        )
        self.assertEqual(args.radar_center_behind_a_cm, 4.0)
        self.assertTrue(args.no_camera_correction)
        self.assertTrue(args.fleet_position_only)

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

    def test_fleet_state_reports_fresh_start_relative_pose(self):
        app = RadarCameraLineApplication(MainConfig())
        now = time.monotonic()
        with app._lock:
            app._ready = True
            app._map_ready = True
            app._latest_navigation_pose = NavigationPose(
                12.4,
                -8.6,
                359.99,
                now,
            )
            app._localization_degraded = False
            app._follower_state = TrackFollowerState(
                running=True,
                completed=False,
                segment=TrackSegment.AB,
                progress_cm=50.0,
                target_speed_cm_s=30.0,
                commanded_speed_cm_s=30.0,
                steering_angle_rad=0.0,
                cross_track_error_cm=0.0,
                heading_error_deg=0.0,
            )

        state = app._fleet_state()

        self.assertEqual((state.x_cm, state.y_cm), (12, -9))
        self.assertEqual(state.heading_cdeg, 35999)
        self.assertTrue(state.node_flags & int(NodeFlags.POSE_VALID))
        self.assertTrue(state.node_flags & int(NodeFlags.READY))
        self.assertTrue(state.node_flags & int(NodeFlags.MAP_READY))
        self.assertTrue(state.node_flags & int(NodeFlags.BUSY))
        self.assertTrue(
            state.node_flags & int(NodeFlags.ARMED_OR_MOTOR_ACTIVE)
        )
        self.assertEqual(state.operation_state, 4)
        self.assertEqual(state.pose_quality, 4)

    def test_fleet_state_marks_stale_pose_degraded_without_inventing_zero(self):
        app = RadarCameraLineApplication(MainConfig())
        with app._lock:
            app._ready = True
            app._map_ready = True
            app._latest_navigation_pose = NavigationPose(
                7.0,
                8.0,
                90.0,
                time.monotonic() - 2.0,
            )
            app._localization_degraded = False

        state = app._fleet_state()

        self.assertEqual((state.x_cm, state.y_cm), (7, 8))
        self.assertFalse(state.node_flags & int(NodeFlags.POSE_VALID))
        self.assertTrue(
            state.node_flags & int(NodeFlags.LOCALIZATION_DEGRADED)
        )
        self.assertEqual(state.pose_quality, 2)

    def test_fixed_track_fleet_commands_are_read_only_except_stop(self):
        app = RadarCameraLineApplication(MainConfig())

        unsupported = app._fleet_unsupported(object())
        stopped = app._fleet_stop()

        self.assertEqual(unsupported.status, AckStatus.REJECTED)
        self.assertEqual(unsupported.reason, AckReason.UNSUPPORTED)
        self.assertEqual(stopped.status, AckStatus.COMPLETED)
        self.assertTrue(app._stop_event.is_set())

    def test_task1_waits_for_ready_start_command_and_reports_request(self):
        app = RadarCameraLineApplication(
            MainConfig(
                fleet_wait_for_start=True,
                fleet_mission_request_state=13,
            )
        )
        self.assertEqual(13, app._fleet_state().operation_state)
        rejected = app._fleet_start_mission()
        self.assertEqual(AckStatus.REJECTED, rejected.status)
        self.assertEqual(AckReason.NOT_READY, rejected.reason)

        with app._lock:
            app._ready = True
        completed = app._fleet_start_mission()
        self.assertEqual(AckStatus.COMPLETED, completed.status)
        self.assertTrue(app._mission_start_event.is_set())
        self.assertEqual(2, app._fleet_state().operation_state)

    def test_invalid_values_are_rejected(self):
        with self.assertRaises(ValueError):
            MainConfig(ab_speed_cm_s=0.0)
        with self.assertRaises(ValueError):
            MainConfig(bc_speed_cm_s=float("nan"))
        with self.assertRaises(ValueError):
            MainConfig(radar_center_behind_a_cm=-0.1)
        with self.assertRaises(ValueError):
            MainConfig(camera_source=-1)
        with self.assertRaises(ValueError):
            MainConfig(
                fleet_position_reporting_enabled=False,
                fleet_position_only=True,
            )
        with self.assertRaises(ValueError):
            MainConfig(
                fleet_position_reporting_enabled=False,
                fleet_wait_for_start=True,
            )


if __name__ == "__main__":
    unittest.main()
