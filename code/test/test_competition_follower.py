from types import SimpleNamespace
import unittest

from components.competition_track import (
    CompetitionTrack,
    CompetitionTrackFollower,
    CompetitionTrackSpeedProfile,
    FINISH_APPROACH_DISTANCE_CM,
    FINISH_APPROACH_SPEED_CM_S,
    FINISH_HEADING_TOLERANCE_DEG,
    FINISH_MAX_OVERSHOOT_CM,
    S_FINISH_CM,
    TrackSegment,
)
from components.navigation import (
    NavigationPose,
    TrackerCommand,
    navigation_heading_to_radar_yaw,
)
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
    def __init__(
        self,
        *,
        index,
        steering_angle_rad,
        cross_track_error_cm=2.5,
        heading_error_deg=4.0,
    ):
        self.index = index
        self.steering_angle_rad = steering_angle_rad
        self.cross_track_error_cm = cross_track_error_cm
        self.heading_error_deg = heading_error_deg
        self.calls = []

    def compute(self, pose, path, *, min_path_index=0):
        self.calls.append((pose, path, min_path_index))
        return TrackerCommand(
            speed_mm_s=9999.0,
            steering_angle_rad=self.steering_angle_rad,
            direction=MotorDirection.FORWARD,
            nearest_path_index=self.index,
            cross_track_error_cm=self.cross_track_error_cm,
            distance_to_goal_cm=500.0,
            signed_cross_track_error_cm=-self.cross_track_error_cm,
            heading_error_deg=self.heading_error_deg,
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
            reference_offset_cm=0.0
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

    def test_optional_control_pose_does_not_replace_radar_acceptance(self):
        controller = FakeController(index=10, steering_angle_rad=-0.123)
        follower = CompetitionTrackFollower(
            drive=self.drive,
            track=self.track,
            speed_cm_s=TEST_SPEED_CM_S,
            controller=controller,
        )
        follower.start_mission()
        override = NavigationPose(12.0, 7.0, 4.0, 123.0)

        follower.update_from_radar(
            radar_update(self.track, 10),
            control_pose_override=override,
        )
        self.assertIs(controller.calls[-1][0], override)

        calls_before = len(controller.calls)
        follower.update_from_radar(
            radar_update(self.track, 11, accepted=False),
            control_pose_override=NavigationPose(99.0, 99.0, 99.0),
        )
        self.assertEqual(len(controller.calls), calls_before)

    def test_segment_profile_changes_speed_without_stopping_at_corners(self):
        profile = CompetitionTrackSpeedProfile(6.0, 7.0, 8.0, 9.0)
        follower = CompetitionTrackFollower(
            drive=self.drive,
            track=self.track,
            speed_profile=profile,
        )
        follower.start_mission()
        for index, expected_speed in zip(
            self.track.segment_start_indices,
            (6.0, 7.0, 8.0, 9.0),
        ):
            follower._progress_index = max(0, index - 1)
            state = follower.update_from_radar(radar_update(self.track, index))
            self.assertEqual(state.target_speed_cm_s, expected_speed)
            self.assertEqual(self.drive.commands[-1][0], expected_speed * 10.0)
        self.assertEqual(self.drive.stops, 0)

    def test_terminal_approach_speed_ramps_to_eight_cm_s(self):
        finish_index = next(
            index
            for index, point in enumerate(self.track.points)
            if point.progress_cm >= self.track.finish_progress_cm
        )
        approach_index = next(
            index
            for index, point in enumerate(self.track.points)
            if point.progress_cm
            >= self.track.finish_progress_cm
            - FINISH_APPROACH_DISTANCE_CM / 2.0
        )
        controller = FakeController(
            index=approach_index,
            steering_angle_rad=0.0,
        )
        follower = CompetitionTrackFollower(
            drive=self.drive,
            track=self.track,
            speed_cm_s=30.0,
            controller=controller,
        )
        follower.start_mission()

        state = follower.update_from_radar(
            radar_update(self.track, approach_index)
        )
        self.assertGreater(state.target_speed_cm_s, FINISH_APPROACH_SPEED_CM_S)
        self.assertLess(state.target_speed_cm_s, 30.0)

        controller.index = finish_index
        controller.cross_track_error_cm = 4.0
        state = follower.update_from_radar(
            radar_update(self.track, finish_index)
        )
        self.assertTrue(state.running)
        self.assertEqual(
            state.target_speed_cm_s,
            FINISH_APPROACH_SPEED_CM_S,
        )

    def test_completion_waits_for_terminal_position_heading_and_cross_track(self):
        finish_index = next(
            index
            for index, point in enumerate(self.track.points)
            if point.progress_cm >= self.track.finish_progress_cm
        )
        controller = FakeController(
            index=finish_index,
            steering_angle_rad=0.0,
            cross_track_error_cm=4.0,
            heading_error_deg=8.0,
        )
        follower = CompetitionTrackFollower(
            drive=self.drive,
            track=self.track,
            speed_cm_s=30.0,
            controller=controller,
        )
        follower.start_mission()

        outside = follower.update_from_radar(
            radar_update(self.track, finish_index)
        )
        self.assertTrue(outside.running)
        self.assertFalse(outside.completed)
        self.assertEqual(self.drive.stops, 0)

        controller.cross_track_error_cm = 2.0
        controller.heading_error_deg = 2.0
        inside = follower.update_from_radar(
            radar_update(self.track, finish_index)
        )
        self.assertFalse(inside.running)
        self.assertTrue(inside.completed)
        self.assertTrue(follower.terminal_tolerance_met)
        self.assertEqual(FINISH_HEADING_TOLERANCE_DEG, 3.0)
        self.assertEqual(self.drive.stops, 1)

    def test_terminal_hard_stop_prevents_unbounded_overshoot(self):
        hard_stop_index = next(
            index
            for index, point in enumerate(self.track.points)
            if point.progress_cm
            >= self.track.finish_progress_cm + FINISH_MAX_OVERSHOOT_CM
        )
        controller = FakeController(
            index=hard_stop_index,
            steering_angle_rad=0.0,
            cross_track_error_cm=10.0,
            heading_error_deg=20.0,
        )
        follower = CompetitionTrackFollower(
            drive=self.drive,
            track=self.track,
            speed_cm_s=30.0,
            controller=controller,
        )
        follower.start_mission()

        state = follower.update_from_radar(
            radar_update(self.track, hard_stop_index)
        )
        self.assertFalse(state.running)
        self.assertTrue(state.completed)
        self.assertFalse(follower.terminal_tolerance_met)
        self.assertTrue(follower.terminal_hard_stop_triggered)
        self.assertEqual(self.drive.stops, 1)

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

    def test_finish_extension_keeps_following_until_rear_axle_reaches_a(self):
        extension_cm = 24.0
        track = CompetitionTrack.build(
            reference_offset_cm=extension_cm,
            finish_extension_cm=extension_cm,
        )
        controller = FakeController(
            index=track.wrap_start_index,
            steering_angle_rad=0.0,
            heading_error_deg=2.0,
        )
        follower = CompetitionTrackFollower(
            drive=self.drive,
            track=track,
            speed_cm_s=TEST_SPEED_CM_S,
            controller=controller,
        )
        follower.start_mission()

        at_front_reference_a = follower.update_from_radar(
            radar_update(track, track.wrap_start_index)
        )

        self.assertTrue(at_front_reference_a.running)
        self.assertFalse(at_front_reference_a.completed)
        self.assertEqual(at_front_reference_a.segment, TrackSegment.AB)
        self.assertEqual(self.drive.stops, 0)

        finish_index = next(
            index
            for index, point in enumerate(track.points)
            if point.progress_cm >= track.finish_progress_cm
        )
        controller.index = finish_index
        at_rear_axle_a = follower.update_from_radar(
            radar_update(track, finish_index)
        )

        self.assertFalse(at_rear_axle_a.running)
        self.assertTrue(at_rear_axle_a.completed)
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
