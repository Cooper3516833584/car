from types import SimpleNamespace
import unittest

from components.competition_track import (
    CompetitionTrack,
    CompetitionTrackFollower,
    S_FINISH_CM,
    TRACK_REFERENCE_OFFSET_CM,
    TrackSegment,
)
from components.navigation import TrackerCommand, navigation_heading_to_radar_yaw
from components.rear_motor import MotorDirection
from components.radar_driver import (
    Pose2D,
    RadarLocalizationUpdate,
    RadarOdometryUpdate,
    RadarScan,
)

TEST_SPEED_CM_S = 8.0


class FakeDrive:
    def __init__(self):
        self.commands = []
        self.stops = 0

    def set_motion(
        self,
        speed,
        steering,
        *,
        direction,
        rear_differential_linked,
    ):
        self.commands.append(
            (speed, steering, direction, rear_differential_linked)
        )
        return SimpleNamespace(center_speed_mm_s=speed)

    def stop(self, *, center_steering=True):
        self.stops += 1


class FakeController:
    def __init__(self, *, index, steering_angle_rad):
        self.index = index
        self.steering_angle_rad = steering_angle_rad
        self.calls = []

    def compute(self, pose, path, *, min_path_index=0):
        self.calls.append((pose, path, min_path_index))
        return TrackerCommand(
            speed_mm_s=9999.0,
            steering_angle_rad=self.steering_angle_rad,
            direction=MotorDirection.FORWARD,
            nearest_path_index=self.index,
            cross_track_error_cm=2.5,
            distance_to_goal_cm=500.0,
            signed_cross_track_error_cm=-2.5,
            heading_error_deg=4.0,
        )


def radar_update(track, index, *, accepted=True, global_pose=True):
    point = track.point_at_index(index)
    pose = Pose2D(
        point.x_cm,
        point.y_cm,
        navigation_heading_to_radar_yaw(point.heading_deg),
    )
    return RadarLocalizationUpdate(
        RadarScan((), 0, 36000),
        RadarOdometryUpdate(pose, accepted, True),
        pose if global_pose else None,
    )


class CompetitionTrackFollowerTests(unittest.TestCase):
    def setUp(self):
        self.drive = FakeDrive()
        self.track = CompetitionTrack.build(
            reference_offset_cm=TRACK_REFERENCE_OFFSET_CM
        )
        self.states = []
        self.follower = CompetitionTrackFollower(
            drive=self.drive,
            track=self.track,
            speed_cm_s=TEST_SPEED_CM_S,
            on_state_changed=self.states.append,
        )
        self.follower.start_mission()

    def test_b_c_d_change_segment_without_stopping(self):
        for index, expected in zip(
            self.track.segment_start_indices,
            (
                TrackSegment.AB,
                TrackSegment.BC,
                TrackSegment.CD,
                TrackSegment.DA,
            ),
        ):
            self.follower._progress_index = max(0, index - 1)
            state = self.follower.update_from_radar(
                radar_update(self.track, index)
            )
            self.assertEqual(state.segment, expected)
        self.assertEqual(self.drive.stops, 0)
        self.assertTrue(
            all(
                command[2] is MotorDirection.FORWARD and command[3]
                for command in self.drive.commands
            )
        )

    def test_progress_index_never_decreases(self):
        later = self.track.segment_start_indices[2] + 5
        self.follower._progress_index = later
        self.follower._state = self.follower._state.__class__(
            True,
            False,
            TrackSegment.CD,
            self.track.point_at_index(later).progress_cm,
            18.0,
            18.0,
            0.0,
            0.0,
            0.0,
        )
        before = self.follower.state.progress_cm
        state = self.follower.update_from_radar(
            radar_update(self.track, later - 3)
        )
        self.assertGreaterEqual(state.progress_cm, before)
        self.assertGreaterEqual(self.follower.progress_index, later)

    def test_invalid_radar_update_does_not_command_drive(self):
        before = self.follower.state
        rejected = self.follower.update_from_radar(
            radar_update(self.track, 5, accepted=False)
        )
        missing = self.follower.update_from_radar(
            radar_update(self.track, 5, global_pose=False)
        )
        self.assertEqual(rejected, before)
        self.assertEqual(missing, before)
        self.assertEqual(self.drive.commands, [])

    def test_fixed_speed_and_pure_pursuit_steering_are_used(self):
        controller = FakeController(index=10, steering_angle_rad=-0.123)
        follower = CompetitionTrackFollower(
            drive=self.drive,
            track=self.track,
            speed_cm_s=TEST_SPEED_CM_S,
            controller=controller,
        )
        follower.start_mission()
        state = follower.update_from_radar(radar_update(self.track, 10))
        self.assertEqual(
            self.drive.commands[-1][0],
            TEST_SPEED_CM_S * 10.0,
        )
        self.assertNotEqual(self.drive.commands[-1][0], 9999.0)
        self.assertEqual(self.drive.commands[-1][1], -0.123)
        self.assertEqual(state.steering_angle_rad, -0.123)
        self.assertEqual(state.commanded_speed_cm_s, TEST_SPEED_CM_S)

    def test_start_does_not_misclassify_completion(self):
        self.assertTrue(self.follower.state.running)
        self.assertFalse(self.follower.state.completed)
        self.assertEqual(self.follower.state.progress_cm, 0.0)
        self.assertEqual(self.drive.stops, 0)

    def test_completion_requires_finish_progress_and_wrapped_index(self):
        wrap = self.track.wrap_start_index
        self.follower._progress_index = wrap - 1
        self.follower._state = self.follower._state.__class__(
            True,
            False,
            TrackSegment.DA,
            S_FINISH_CM,
            8.0,
            8.0,
            0.0,
            0.0,
            0.0,
        )
        state = self.follower.update_from_radar(
            radar_update(self.track, wrap - 1)
        )
        self.assertTrue(state.running)
        self.assertFalse(state.completed)
        self.assertEqual(self.drive.stops, 0)

        self.follower._progress_index = wrap
        state = self.follower.update_from_radar(
            radar_update(self.track, wrap)
        )
        self.assertFalse(state.running)
        self.assertTrue(state.completed)
        self.assertEqual(self.drive.stops, 1)

        self.follower.update_from_radar(radar_update(self.track, wrap + 1))
        self.assertEqual(self.drive.stops, 1)

    def test_failure_stops_car(self):
        self.drive.set_motion = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("write failed")
        )
        with self.assertRaises(RuntimeError):
            self.follower.update_from_radar(radar_update(self.track, 5))
        self.assertFalse(self.follower.state.running)
        self.assertEqual(self.drive.stops, 1)


if __name__ == "__main__":
    unittest.main()
