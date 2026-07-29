import math
import unittest

from components.competition_track import (
    A_FIELD_CM,
    B_FIELD_CM,
    C_FIELD_CM,
    D_FIELD_CM,
    S_B_CM,
    S_C_CM,
    S_D_CM,
    S_FINISH_CM,
    TRACK_RADIUS_CM,
    TRACK_SAMPLE_SPACING_CM,
    WRAP_EXTENSION_CM,
    CompetitionTrack,
    TrackSegment,
    build_competition_track,
)
from components.rear_motor import MotorDirection


class CompetitionTrackGeometryTests(unittest.TestCase):
    def test_field_key_points_and_lap_length(self):
        track = build_competition_track(reference_offset_cm=0.0)
        a, b, c, d = (
            track.field_points_cm[track.segment_start_indices[index]]
            for index in range(4)
        )
        self.assertEqual(a, A_FIELD_CM)
        self.assertEqual(b, B_FIELD_CM)
        self.assertAlmostEqual(c[0], C_FIELD_CM[0])
        self.assertAlmostEqual(c[1], C_FIELD_CM[1])
        self.assertAlmostEqual(d[0], D_FIELD_CM[0])
        self.assertAlmostEqual(d[1], D_FIELD_CM[1])
        self.assertAlmostEqual(S_FINISH_CM, 771.238898038469)
        self.assertEqual(S_B_CM, 150.0)
        self.assertEqual(S_C_CM, 150.0 + math.pi * 75.0)
        self.assertEqual(S_D_CM, 300.0 + math.pi * 75.0)
        self.assertEqual(S_FINISH_CM, 300.0 + 2.0 * math.pi * 75.0)

    def test_connections_and_headings_are_continuous(self):
        track = build_competition_track(reference_offset_cm=0.0)
        expected = (A_FIELD_CM, B_FIELD_CM, C_FIELD_CM, D_FIELD_CM)
        for index, coordinate in zip(track.segment_start_indices, expected):
            actual = track.field_points_cm[index]
            self.assertAlmostEqual(actual[0], coordinate[0])
            self.assertAlmostEqual(actual[1], coordinate[1])
        for first, second in zip(track.points, track.points[1:]):
            heading_step = (
                second.heading_deg - first.heading_deg + 180.0
            ) % 360.0 - 180.0
            self.assertLessEqual(abs(heading_step), 2.0)

    def test_arcs_have_75_cm_field_radius(self):
        track = build_competition_track(reference_offset_cm=0.0)
        starts = track.segment_start_indices
        for x_cm, y_cm in track.field_points_cm[starts[1] : starts[2] + 1]:
            self.assertAlmostEqual(
                math.hypot(x_cm - 225.0, y_cm - 350.0),
                TRACK_RADIUS_CM,
                places=6,
            )
        for x_cm, y_cm in track.field_points_cm[
            starts[3] : track.wrap_start_index + 1
        ]:
            self.assertAlmostEqual(
                math.hypot(x_cm - 225.0, y_cm - 200.0),
                TRACK_RADIUS_CM,
                places=6,
            )

    def test_default_sampling_and_wrapped_progress(self):
        track = build_competition_track()
        self.assertAlmostEqual(
            track.point_at_index(track.wrap_start_index).progress_cm,
            S_FINISH_CM,
        )
        self.assertAlmostEqual(
            track.points[-1].progress_cm,
            S_FINISH_CM + WRAP_EXTENSION_CM,
        )
        steps = [
            second.progress_cm - first.progress_cm
            for first, second in zip(track.points, track.points[1:])
        ]
        self.assertLessEqual(max(steps), TRACK_SAMPLE_SPACING_CM + 1e-9)
        self.assertGreaterEqual(min(steps), 0.0)
        self.assertGreater(len(track.points) - track.wrap_start_index, 1)

    def test_rear_axle_at_a_is_rebased_to_local_origin(self):
        track = CompetitionTrack.build(reference_offset_cm=0.0)
        self.assertAlmostEqual(track.field_points_cm[0][0], 150.0)
        self.assertAlmostEqual(track.field_points_cm[0][1], 200.0)
        start = track.point_at_index(0)
        self.assertAlmostEqual(start.x_cm, 0.0)
        self.assertAlmostEqual(start.y_cm, 0.0)
        self.assertAlmostEqual(start.heading_deg, 0.0)

    def test_navigation_path_is_forward_and_public_accessors_match(self):
        track = CompetitionTrack.build(reference_offset_cm=0.0)
        self.assertEqual(len(track.points), len(track.path.points))
        self.assertTrue(
            all(
                point.direction is MotorDirection.FORWARD
                for point in track.path.points
            )
        )
        self.assertEqual(track.point_at_index(0), track.points[0])
        self.assertEqual(track.finish_progress_cm, S_FINISH_CM)
        self.assertEqual(track.segment_at_progress(0.0), TrackSegment.AB)
        self.assertEqual(track.segment_at_progress(S_B_CM), TrackSegment.BC)
        self.assertEqual(track.segment_at_progress(S_C_CM), TrackSegment.CD)
        self.assertEqual(track.segment_at_progress(S_D_CM), TrackSegment.DA)
        self.assertEqual(
            track.segment_at_progress(S_FINISH_CM), TrackSegment.AB
        )


if __name__ == "__main__":
    unittest.main()
