import math
from dataclasses import replace
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from camera_line_follower import (
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


def observation(*, y_left=0.0, confidence=0.9, detected=True, timestamp=10.0):
    # y(x) = constant lateral offset.
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
        polynomial_y_left_by_x=(0.0, 0.0, y_left) if detected else None,
        dark_threshold=90.0,
    )


@unittest.skipIf(__import__("camera_line_follower").cv2 is None, "OpenCV unavailable")
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

    def test_curve_polynomial_has_expected_sign(self):
        result = self.detector.process(self.synthetic_frame(curve=0.002), timestamp_s=1.0)
        self.assertTrue(result.detected)
        self.assertGreater(result.heading_error_rad, 0.0)


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

    def test_steering_rate_is_limited(self):
        first = self.follower.compute_command(observation(y_left=20.0), now_s=10.0)
        second = self.follower.compute_command(observation(y_left=-20.0), now_s=10.01)
        self.assertLessEqual(
            abs(second.steering_angle_rad - first.steering_angle_rad),
            self.follower.control_config.maximum_steering_rate_rad_s * 0.01 + 1e-9,
        )


if __name__ == "__main__":
    unittest.main()
