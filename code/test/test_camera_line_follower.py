import math
from dataclasses import replace
import unittest
from types import SimpleNamespace

import numpy as np

from components import camera_line_follower
from components.camera_line_follower import (
    BlackLineDetector,
    CameraLineFollower,
    LineControlConfig,
    LineFollowerStatus,
    LineObservation,
    LineVisionConfig,
    PerspectiveConfig,
)


class FakeDrive:
    def __init__(self):
        self.is_running = True
        self.commands = []
        self.stops = 0

    def set_motion(self, speed, steering, *, direction, rear_differential_linked):
        self.commands.append((speed, steering, direction, rear_differential_linked))
        return SimpleNamespace(center_speed_mm_s=speed)

    def stop(self, *, center_steering=True):
        self.stops += 1


def observation(
    *,
    y_left=0.0,
    confidence=0.9,
    detected=True,
    timestamp=10.0,
    polynomial=None,
    forward_heading_change_rad=0.0,
):
    # y(x) = constant lateral offset.
    if polynomial is None and detected:
        polynomial = (0.0, 0.0, y_left)
    return LineObservation(
        timestamp_s=timestamp,
        detected=detected,
        confidence=confidence,
        lookahead_x_cm=35.0,
        lookahead_y_left_cm=y_left,
        near_lateral_error_cm=y_left,
        heading_error_rad=0.0,
        curvature_per_cm=0.0,
        fit_rmse_cm=0.5,
        visible_band_count=12,
        total_band_count=13,
        median_line_width_cm=5.0,
        polynomial_y_left_by_x=polynomial if detected else None,
        dark_threshold=90.0,
        forward_heading_change_rad=forward_heading_change_rad,
    )


@unittest.skipIf(camera_line_follower.cv2 is None, "OpenCV unavailable")
class DetectorTests(unittest.TestCase):
    def setUp(self):
        perspective = PerspectiveConfig(
            source_points_norm=((0.0, 1.0), (1.0, 1.0), (1.0, 0.0), (0.0, 0.0)),
            output_width_px=320,
            output_height_px=400,
            ground_width_cm=80.0,
            ground_depth_cm=100.0,
        )
        self.config = LineVisionConfig(
            frame_width=320,
            frame_height=400,
            perspective=perspective,
            scan_near_cm=8.0,
            scan_far_cm=78.0,
        )
        self.detector = BlackLineDetector(self.config)

    def synthetic_frame(self, lateral_cm=0.0, curve=0.0):
        import cv2

        image = np.full((400, 320, 3), 225, dtype=np.uint8)
        cm_per_px_x = 80.0 / 320.0
        cm_per_px_y = 100.0 / 400.0
        points = []
        for x_forward in np.linspace(0.0, 95.0, 140):
            y_left = lateral_cm + curve * x_forward * x_forward
            u = int(round(159.5 - y_left / cm_per_px_x))
            v = int(round(399 - x_forward / cm_per_px_y))
            points.append((u, v))
        cv2.polylines(image, [np.asarray(points, np.int32)], False, (20, 20, 20), 20)
        # Add a broad soft shadow that should not replace the thin line.
        cv2.rectangle(image, (0, 80), (100, 250), (150, 150, 150), -1)
        return image

    def test_straight_line_is_detected(self):
        result = self.detector.process(self.synthetic_frame(), timestamp_s=1.0)
        self.assertTrue(result.detected)
        self.assertGreater(result.confidence, 0.55)
        self.assertAlmostEqual(result.lookahead_y_left_cm, 0.0, delta=2.0)

    def test_left_line_has_positive_vehicle_y(self):
        result = self.detector.process(self.synthetic_frame(lateral_cm=8.0), timestamp_s=1.0)
        self.assertTrue(result.detected)
        self.assertGreater(result.lookahead_y_left_cm, 4.0)

    def test_wide_horizontal_black_line_is_a_transverse_stop_marker(self):
        import cv2

        image = np.full((400, 320, 3), 225, dtype=np.uint8)
        # 5 cm thick marker that spans 70 cm of the calibrated 80 cm view.
        cv2.rectangle(image, (20, 190), (300, 210), (20, 20, 20), -1)
        result = self.detector.process(image, timestamp_s=1.0)
        self.assertTrue(result.transverse_line_detected)
        self.assertFalse(result.round_marker_detected)

    def test_large_round_track_dot_is_not_a_finish_line(self):
        rows, columns = np.ogrid[:400, :320]
        mask = np.zeros((400, 320), dtype=np.uint8)
        ellipse = (
            ((columns - 160.0) / 125.0) ** 2
            + ((rows - 330.0) / 75.0) ** 2
            <= 1.0
        )
        mask[ellipse] = 255
        transverse, round_marker = self.detector._classify_track_markers(mask)
        self.assertFalse(transverse)
        self.assertTrue(round_marker)

    def test_curve_polynomial_has_expected_sign(self):
        result = self.detector.process(self.synthetic_frame(curve=0.002), timestamp_s=1.0)
        self.assertTrue(result.detected)
        self.assertGreater(result.heading_error_rad, 0.0)

    def test_new_curve_is_followed_from_near_to_far_in_current_frame(self):
        self.detector.process(self.synthetic_frame(), timestamp_s=1.0)
        result = self.detector.process(
            self.synthetic_frame(curve=0.006),
            timestamp_s=1.1,
        )

        self.assertTrue(result.detected)
        self.assertGreaterEqual(result.visible_band_count, 8)
        self.assertGreater(result.lookahead_y_left_cm, 1.5)
        self.assertGreater(result.forward_heading_change_rad, 0.10)


class BandTrackingTests(unittest.TestCase):
    def test_rows_follow_current_curve_instead_of_previous_straight_fit(self):
        perspective = PerspectiveConfig(
            source_points_norm=((0.0, 1.0), (1.0, 1.0), (1.0, 0.0), (0.0, 0.0)),
            output_width_px=320,
            output_height_px=400,
            ground_width_cm=80.0,
            ground_depth_cm=100.0,
        )
        config = LineVisionConfig(
            perspective=perspective,
            scan_near_cm=8.0,
            scan_far_cm=78.0,
            maximum_center_jump_cm=8.0,
        )
        detector = BlackLineDetector(config)
        detector._previous_polynomial = np.asarray((0.0, 0.0, 0.0))
        mask = np.zeros((400, 320), dtype=np.uint8)
        for row in range(400):
            forward_cm = (399 - row) * 0.25
            y_left_cm = 0.006 * forward_cm * forward_cm
            centre = int(round(159.5 - y_left_cm / 0.25))
            start = max(0, centre - 10)
            end = min(320, centre + 10)
            mask[row, start:end] = 255

        points, _ = detector._extract_band_centres(
            mask,
            np.where(mask > 0, 20, 225).astype(np.uint8),
        )

        self.assertEqual(len(points), config.scan_count)
        self.assertGreater(points[-1][1], 30.0)

    def test_round_marker_rows_are_excluded_and_far_track_can_seed_startup(self):
        perspective = PerspectiveConfig(
            source_points_norm=((0.0, 1.0), (1.0, 1.0), (1.0, 0.0), (0.0, 0.0)),
            output_width_px=320,
            output_height_px=400,
            ground_width_cm=80.0,
            ground_depth_cm=100.0,
        )
        config = LineVisionConfig(
            perspective=perspective,
            scan_near_cm=8.0,
            scan_far_cm=78.0,
        )
        detector = BlackLineDetector(config)
        mask = np.zeros((400, 320), dtype=np.uint8)
        mask[:, 150:170] = 255
        mask[270:330, 20:300] = 255

        points, widths = detector._extract_band_centres(
            mask,
            np.where(mask > 0, 20, 225).astype(np.uint8),
            ignore_wide_bands=True,
        )
        result = detector._fit_observation(
            points,
            widths,
            timestamp_s=1.0,
            dark_threshold=90.0,
            minimum_fit_points=3,
        )

        self.assertGreaterEqual(len(points), 3)
        self.assertLess(len(points), config.scan_count)
        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.lookahead_y_left_cm, 0.0, delta=1.0)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.drive = FakeDrive()
        self.follower = CameraLineFollower(
            drive=self.drive,
            vision_config=LineVisionConfig(),
            control_config=LineControlConfig(recovery_good_frames=1),
        )

    def test_line_on_left_commands_positive_left_steering(self):
        command = self.follower.compute_command(observation(y_left=8.0), now_s=10.0)
        self.assertTrue(command.moving)
        self.assertEqual(command.status, LineFollowerStatus.TRACKING)
        self.assertGreater(command.steering_angle_rad, 0.0)

    def test_line_on_right_commands_negative_steering(self):
        command = self.follower.compute_command(observation(y_left=-8.0), now_s=10.0)
        self.assertLess(command.steering_angle_rad, 0.0)

    def test_sustained_loss_stops(self):
        lost = observation(detected=False, confidence=0.0)
        command = None
        for i in range(5):
            command = self.follower.compute_command(lost, now_s=10.0 + i * 0.03)
        self.assertIsNotNone(command)
        self.assertFalse(command.moving)
        self.assertEqual(command.status, LineFollowerStatus.LOST)

    def test_stale_frame_is_not_trusted(self):
        command = self.follower.compute_command(observation(timestamp=1.0), now_s=10.0)
        self.assertNotEqual(command.status, LineFollowerStatus.TRACKING)

    def test_start_line_is_ignored_until_it_clears_then_finish_stops(self):
        follower = CameraLineFollower(
            drive=self.drive,
            control_config=LineControlConfig(
                finish_line_startup_grace_s=0.0,
                finish_line_clear_frames_to_arm=2,
                finish_line_confirm_frames=2,
                minimum_markers_before_finish=0,
            ),
        )
        follower._run_started_at_s = 0.0
        marker = replace(observation(), transverse_line_detected=True)
        clear = observation()

        self.assertFalse(follower._finish_line_reached(marker, 1.0))
        self.assertFalse(follower._finish_line_reached(clear, 1.1))
        self.assertFalse(follower._finish_line_reached(clear, 1.2))
        self.assertTrue(follower._finish_line_armed)
        self.assertFalse(follower._finish_line_reached(marker, 1.3))
        self.assertTrue(follower._finish_line_reached(marker, 1.4))

    def test_round_marker_counts_once_after_startup_dot_clears(self):
        counts = []
        follower = CameraLineFollower(
            drive=self.drive,
            control_config=LineControlConfig(
                finish_line_startup_grace_s=0.0,
                round_marker_clear_frames_to_arm=2,
                round_marker_confirm_frames=2,
            ),
            on_marker_passed=counts.append,
        )
        follower._run_started_at_s = 0.0
        dot = replace(observation(), round_marker_detected=True)
        clear = observation()

        follower._update_round_marker(dot, 1.0)
        follower._update_round_marker(clear, 1.1)
        follower._update_round_marker(clear, 1.2)
        follower._update_round_marker(dot, 1.3)
        follower._update_round_marker(dot, 1.4)
        follower._update_round_marker(dot, 1.5)
        self.assertEqual(counts, [1])
        self.assertEqual(follower._marker_count, 1)

    def test_fourth_round_marker_is_a_finish_fallback_at_a(self):
        self.follower._run_started_at_s = 0.0
        self.follower._finish_line_armed = True
        self.follower._marker_count = 4
        self.assertTrue(
            self.follower._finish_line_reached(observation(), 2.0)
        )

    def test_steering_rate_is_limited(self):
        first = self.follower.compute_command(observation(y_left=20.0), now_s=10.0)
        second = self.follower.compute_command(observation(y_left=-20.0), now_s=10.01)
        self.assertLessEqual(
            abs(second.steering_angle_rad - first.steering_angle_rad),
            self.follower.control_config.maximum_steering_rate_rad_s * 0.01 + 1e-9,
        )

    def test_circle_target_uses_ackermann_pure_pursuit_steering(self):
        radius_cm = 75.0
        follower = CameraLineFollower(
            drive=FakeDrive(),
            control_config=LineControlConfig(
                recovery_good_frames=1,
                steering_low_pass_time_constant_s=0.0,
                maximum_steering_rate_rad_s=100.0,
            ),
        )
        curve = observation(
            polynomial=(1.0 / (2.0 * radius_cm), 0.0, 0.0),
            forward_heading_change_rad=0.6,
        )

        command = follower.compute_command(curve, now_s=10.0)

        self.assertGreater(command.steering_angle_rad, 0.14)
        self.assertLess(command.steering_angle_rad, 0.23)

    def test_upcoming_curve_slows_before_filtered_steering_catches_up(self):
        config = LineControlConfig(
            recovery_good_frames=1,
            maximum_acceleration_mm_s2=1_000_000.0,
            maximum_deceleration_mm_s2=1_000_000.0,
        )
        straight_follower = CameraLineFollower(
            drive=FakeDrive(),
            control_config=config,
        )
        curve_follower = CameraLineFollower(
            drive=FakeDrive(),
            control_config=config,
        )

        straight = straight_follower.compute_command(
            observation(),
            now_s=10.0,
        )
        curve = curve_follower.compute_command(
            observation(forward_heading_change_rad=0.7),
            now_s=10.0,
        )

        self.assertLess(curve.speed_mm_s, straight.speed_mm_s)
        self.assertAlmostEqual(
            curve.speed_mm_s,
            config.minimum_tracking_speed_mm_s,
        )

    def test_speed_ramps_up_instead_of_starting_abruptly(self):
        command = self.follower.compute_command(observation(), now_s=10.0)

        expected_maximum = (
            self.follower.control_config.maximum_acceleration_mm_s2
            / self.follower.vision_config.camera_fps
        )
        self.assertLessEqual(command.speed_mm_s, expected_maximum + 1e-9)

    def test_single_frame_target_spike_does_not_reverse_steering(self):
        follower = CameraLineFollower(
            drive=FakeDrive(),
            control_config=LineControlConfig(
                recovery_good_frames=1,
                lateral_gain=0.0,
                heading_gain=0.0,
                curvature_feedforward_gain=0.0,
                steering_low_pass_time_constant_s=0.0,
                maximum_steering_rate_rad_s=100.0,
            ),
        )

        before = follower.compute_command(observation(y_left=20.0), now_s=10.0)
        spike = follower.compute_command(observation(y_left=-20.0), now_s=10.01)

        self.assertGreater(before.steering_angle_rad, 0.0)
        self.assertGreater(spike.steering_angle_rad, 0.0)


if __name__ == "__main__":
    unittest.main()
