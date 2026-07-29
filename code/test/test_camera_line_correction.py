import unittest

from components.camera_line_correction import (
    CameraLineCorrectionConfig,
    CameraLineSteeringCorrector,
)
from components.camera_line_follower import LineObservation


def observation(
    *,
    lateral_cm=0.0,
    confidence=0.9,
    detected=True,
    visible_bands=12,
    rmse_cm=0.5,
    round_marker=False,
    transverse=False,
):
    return LineObservation(
        timestamp_s=1.0,
        detected=detected,
        confidence=confidence,
        lookahead_x_cm=35.0,
        lookahead_y_left_cm=lateral_cm,
        near_lateral_error_cm=lateral_cm,
        heading_error_rad=0.0,
        curvature_per_cm=0.0,
        fit_rmse_cm=rmse_cm,
        visible_band_count=visible_bands,
        total_band_count=13,
        median_line_width_cm=28.0,
        polynomial_y_left_by_x=(0.0, 0.0, lateral_cm),
        dark_threshold=100.0,
        round_marker_detected=round_marker,
        transverse_line_detected=transverse,
    )


class CameraLineSteeringCorrectorTests(unittest.TestCase):
    def make_corrector(self, **overrides):
        values = {
            "required_consecutive_frames": 2,
            "correction_filter_time_constant_s": 0.0,
            "maximum_correction_rate_rad_s": 100.0,
        }
        values.update(overrides)
        return CameraLineSteeringCorrector(
            correction_config=CameraLineCorrectionConfig(**values)
        )

    def test_small_error_inside_deadband_does_not_change_steering(self):
        corrector = self.make_corrector()
        for index in range(2):
            state = corrector.update_from_observation(
                observation(lateral_cm=8.0),
                now_s=1.0 + 0.04 * index,
            )

        self.assertTrue(state.active)
        self.assertEqual(state.correction_rad, 0.0)

    def test_large_error_produces_only_bounded_same_sign_correction(self):
        corrector = self.make_corrector(
            steering_gain_rad_per_cm=0.02,
            maximum_abs_correction_rad=0.055,
        )
        for index in range(2):
            left = corrector.update_from_observation(
                observation(lateral_cm=30.0),
                now_s=1.0 + 0.04 * index,
            )
        self.assertEqual(left.correction_rad, 0.055)

        corrector = self.make_corrector(
            steering_gain_rad_per_cm=0.02,
            maximum_abs_correction_rad=0.055,
        )
        for index in range(2):
            right = corrector.update_from_observation(
                observation(lateral_cm=-30.0),
                now_s=1.0 + 0.04 * index,
            )
        self.assertEqual(right.correction_rad, -0.055)

    def test_requires_consecutive_reliable_frames(self):
        corrector = CameraLineSteeringCorrector()
        state = None
        for index in range(3):
            state = corrector.update_from_observation(
                observation(lateral_cm=20.0),
                now_s=1.0 + 0.04 * index,
            )
            self.assertFalse(state.active)

        state = corrector.update_from_observation(
            observation(lateral_cm=20.0),
            now_s=1.12,
        )
        self.assertTrue(state.active)
        self.assertGreater(state.correction_rad, 0.0)

    def test_markers_and_low_quality_frames_disable_new_correction(self):
        for rejected in (
            observation(lateral_cm=20.0, confidence=0.4),
            observation(lateral_cm=20.0, visible_bands=3),
            observation(lateral_cm=20.0, rmse_cm=4.0),
            observation(lateral_cm=20.0, round_marker=True),
            observation(lateral_cm=20.0, transverse=True),
        ):
            corrector = self.make_corrector(
                required_consecutive_frames=1
            )
            state = corrector.update_from_observation(
                rejected,
                now_s=1.0,
            )
            self.assertFalse(state.active)
            self.assertEqual(state.correction_rad, 0.0)

    def test_invalid_frame_fades_existing_correction_toward_zero(self):
        corrector = CameraLineSteeringCorrector(
            correction_config=CameraLineCorrectionConfig(
                required_consecutive_frames=1,
                correction_filter_time_constant_s=0.20,
                maximum_correction_rate_rad_s=1.0,
            )
        )
        active = corrector.update_from_observation(
            observation(lateral_cm=30.0),
            now_s=1.0,
        )
        faded = corrector.update_from_observation(
            observation(lateral_cm=30.0, round_marker=True),
            now_s=1.1,
        )

        self.assertGreater(active.correction_rad, 0.0)
        self.assertFalse(faded.active)
        self.assertGreaterEqual(faded.correction_rad, 0.0)
        self.assertLess(faded.correction_rad, active.correction_rad)

    def test_stale_observation_fades_to_zero(self):
        corrector = self.make_corrector(
            required_consecutive_frames=1,
            stale_timeout_s=0.20,
            stale_fade_out_s=0.30,
        )
        corrector.update_from_observation(
            observation(lateral_cm=30.0),
            now_s=1.0,
        )

        fresh = corrector.correction_for_speed(20.0, now_s=1.1)
        stale = corrector.correction_for_speed(20.0, now_s=1.5)

        self.assertGreater(fresh, 0.0)
        self.assertEqual(stale, 0.0)

    def test_high_speed_reduces_camera_influence(self):
        corrector = self.make_corrector(
            required_consecutive_frames=1,
            full_correction_speed_cm_s=25.0,
            minimum_high_speed_scale=0.5,
        )
        corrector.update_from_observation(
            observation(lateral_cm=30.0),
            now_s=1.0,
        )

        low_speed = corrector.correction_for_speed(20.0, now_s=1.0)
        high_speed = corrector.correction_for_speed(50.0, now_s=1.0)

        self.assertGreater(low_speed, high_speed)
        self.assertAlmostEqual(high_speed, low_speed * 0.5)


if __name__ == "__main__":
    unittest.main()
