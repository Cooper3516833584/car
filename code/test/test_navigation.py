"""Hardware-free tests for Ackermann map navigation."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.navigation import (  # noqa: E402
    DEFAULT_TRACK_WIDTH_MM,
    HybridAStarConfig,
    HybridAStarPlanner,
    Navigation,
    NavigationConfig,
    NavigationGoal,
    NavigationPath,
    NavigationPose,
    NavigationState,
    OccupancyGrid,
    PathNotFoundError,
    PathPoint,
    PlanningCancelledError,
    PurePursuitConfig,
    PurePursuitController,
    VehicleCollisionChecker,
    VehicleGeometry,
    navigation_heading_to_radar_yaw,
    normalize_heading_deg,
    radar_yaw_to_navigation_heading,
    signed_heading_error_deg,
)
from components.radar_driver import (  # noqa: E402
    ICPResult,
    Pose2D,
    RadarLocalizationUpdate,
    RadarOdometryUpdate,
    RadarScan,
)
from components.rear_motor import MotorDirection  # noqa: E402


def open_grid() -> OccupancyGrid:
    return OccupancyGrid.empty(
        resolution_cm=5.0,
        origin_x_cm=-150.0,
        origin_y_cm=-150.0,
        width=80,
        height=80,
        unknown_is_occupied=False,
    )


class CoordinateAndGeometryTests(unittest.TestCase):
    def test_measured_dimensions_are_defaults(self) -> None:
        geometry = VehicleGeometry()
        self.assertAlmostEqual(DEFAULT_TRACK_WIDTH_MM, 117.1)
        self.assertAlmostEqual(geometry.track_width_cm, 11.71)
        self.assertAlmostEqual(geometry.wheelbase_cm, 14.25)
        self.assertAlmostEqual(geometry.body_length_cm, 23.0)
        self.assertAlmostEqual(geometry.body_width_cm, 14.5)
        self.assertAlmostEqual(geometry.rear_axle_to_body_center_cm, 7.125)
        self.assertAlmostEqual(geometry.left_min_turn_radius_cm, 35.0)
        self.assertGreater(geometry.right_min_turn_radius_cm, 35.0)
        self.assertLess(geometry.max_left_steering_rad, 0.49)
        self.assertGreaterEqual(geometry.min_right_steering_rad, -0.32)

    def test_heading_normalization_and_shortest_error(self) -> None:
        self.assertEqual(normalize_heading_deg(-10), 350)
        self.assertEqual(normalize_heading_deg(370), 10)
        self.assertEqual(signed_heading_error_deg(1, 359), 2)
        self.assertEqual(signed_heading_error_deg(359, 1), -2)

    def test_radar_clockwise_yaw_conversion(self) -> None:
        self.assertEqual(radar_yaw_to_navigation_heading(90), 270)
        self.assertEqual(radar_yaw_to_navigation_heading(-90), 90)
        self.assertEqual(navigation_heading_to_radar_yaw(90), -90)
        self.assertEqual(navigation_heading_to_radar_yaw(270), 90)

    def test_rectangular_body_collision_uses_measured_length(self) -> None:
        geometry = VehicleGeometry()
        free = open_grid()
        self.assertTrue(
            VehicleCollisionChecker(free, geometry, safety_margin_cm=0).is_pose_free(
                NavigationPose(0, 0, 0, 0)
            )
        )
        obstacle = OccupancyGrid.from_obstacle_points(
            [(18.0, 0.0)],
            resolution_cm=1.0,
            origin_x_cm=-50,
            origin_y_cm=-50,
            width=100,
            height=100,
        )
        self.assertFalse(
            VehicleCollisionChecker(obstacle, geometry, safety_margin_cm=0).is_pose_free(
                NavigationPose(0, 0, 0, 0)
            )
        )


class HybridAStarTests(unittest.TestCase):
    def test_straight_point_goal(self) -> None:
        path = HybridAStarPlanner().plan(
            NavigationPose(0, 0, 0, 0),
            NavigationGoal(100, 0),
            open_grid(),
        )
        end = path.points[-1]
        self.assertLessEqual(math.hypot(end.x_cm - 100, end.y_cm), 15)
        self.assertTrue(all(point.direction is MotorDirection.FORWARD for point in path.points))

    def test_optional_final_heading_is_planned(self) -> None:
        goal = NavigationGoal(100, 100, final_heading_deg=90, heading_tolerance_deg=10)
        path = HybridAStarPlanner().plan(NavigationPose(0, 0, 0, 0), goal, open_grid())
        end = path.points[-1]
        self.assertLessEqual(math.hypot(end.x_cm - goal.x_cm, end.y_cm - goal.y_cm), 15)
        self.assertLessEqual(abs(signed_heading_error_deg(90, end.heading_deg)), 10)

    def test_problematic_forward_u_turn_uses_fast_collision_checked_dubins_path(self) -> None:
        goal = NavigationGoal(100, 200, final_heading_deg=180)
        started = time.monotonic()

        path = HybridAStarPlanner().plan(
            NavigationPose(0, 0, 0, 0),
            goal,
            open_grid(),
        )

        self.assertLess(time.monotonic() - started, 1.0)
        end = path.points[-1]
        self.assertAlmostEqual(end.x_cm, goal.x_cm, places=3)
        self.assertAlmostEqual(end.y_cm, goal.y_cm, places=3)
        self.assertAlmostEqual(
            signed_heading_error_deg(180, end.heading_deg),
            0.0,
            places=3,
        )
        self.assertTrue(
            all(point.direction is MotorDirection.FORWARD for point in path.points)
        )

    def test_planner_honours_cancellation_before_search(self) -> None:
        with self.assertRaises(PlanningCancelledError):
            HybridAStarPlanner().plan(
                NavigationPose(0, 0, 0, 0),
                NavigationGoal(100, 200, final_heading_deg=180),
                open_grid(),
                should_cancel=lambda: True,
            )

    def test_reverse_switch_enables_straight_backing_in_narrow_map(self) -> None:
        corridor = OccupancyGrid.empty(
            resolution_cm=2.5,
            origin_x_cm=-100,
            origin_y_cm=-12.5,
            width=80,
            height=10,
            unknown_is_occupied=False,
        )
        planner = HybridAStarPlanner(
            config=HybridAStarConfig(
                heading_bins=36,
                primitive_length_cm=10,
                collision_sample_cm=2.5,
                max_expansions=2000,
                safety_margin_cm=0,
            )
        )
        start = NavigationPose(0, 0, 0, 0)
        goal = NavigationGoal(-50, 0, position_tolerance_cm=10)
        with self.assertRaises(PathNotFoundError):
            planner.plan(start, goal, corridor, allow_reverse=False)
        path = planner.plan(start, goal, corridor, allow_reverse=True)
        self.assertTrue(any(point.direction is MotorDirection.REVERSE for point in path.points[1:]))

    def test_starting_inside_obstacle_is_rejected(self) -> None:
        grid = OccupancyGrid.from_obstacle_points(
            [(0, 0)],
            resolution_cm=5,
            origin_x_cm=-100,
            origin_y_cm=-100,
            width=40,
            height=40,
        )
        with self.assertRaisesRegex(PathNotFoundError, "start vehicle footprint"):
            HybridAStarPlanner().plan(
                NavigationPose(0, 0, 0, 0), NavigationGoal(50, 0), grid
            )


class PurePursuitTests(unittest.TestCase):
    def test_left_and_right_path_produce_matching_steering_signs(self) -> None:
        controller = PurePursuitController()
        pose = NavigationPose(0, 0, 0, 0)
        left_path = NavigationPath(
            (PathPoint(0, 0, 0), PathPoint(60, 20, 20)), NavigationGoal(60, 20)
        )
        right_path = NavigationPath(
            (PathPoint(0, 0, 0), PathPoint(60, -20, 340)), NavigationGoal(60, -20)
        )
        self.assertGreater(controller.compute(pose, left_path).steering_angle_rad, 0)
        self.assertLess(controller.compute(pose, right_path).steering_angle_rad, 0)

    def test_reverse_path_uses_reverse_speed_limit(self) -> None:
        controller = PurePursuitController()
        path = NavigationPath(
            (
                PathPoint(0, 0, 0),
                PathPoint(-60, 10, 350, MotorDirection.REVERSE),
            ),
            NavigationGoal(-60, 10),
        )
        command = controller.compute(NavigationPose(0, 0, 0, 0), path)
        self.assertIs(command.direction, MotorDirection.REVERSE)
        self.assertLessEqual(command.speed_mm_s, controller.config.reverse_speed_mm_s)

    def test_reverse_speed_below_approach_floor_is_not_raised(self) -> None:
        controller = PurePursuitController(
            config=PurePursuitConfig(
                cruise_speed_mm_s=500.0,
                max_speed_mm_s=500.0,
                approach_speed_mm_s=50.0,
                reverse_speed_mm_s=30.0,
            )
        )
        path = NavigationPath(
            (
                PathPoint(0, 0, 0),
                PathPoint(-100, 0, 0, MotorDirection.REVERSE),
            ),
            NavigationGoal(-100, 0),
        )

        command = controller.compute(NavigationPose(0, 0, 0, 0), path)

        self.assertIs(command.direction, MotorDirection.REVERSE)
        self.assertEqual(command.speed_mm_s, 30.0)

    def test_speed_reduces_near_goal(self) -> None:
        controller = PurePursuitController()
        far_path = NavigationPath(
            (PathPoint(0, 0, 0), PathPoint(100, 0, 0)), NavigationGoal(100, 0)
        )
        near_path = NavigationPath(
            (PathPoint(80, 0, 0), PathPoint(100, 0, 0)), NavigationGoal(100, 0)
        )
        far = controller.compute(NavigationPose(0, 0, 0, 0), far_path)
        near = controller.compute(NavigationPose(80, 0, 0, 0), near_path)
        self.assertLess(near.speed_mm_s, far.speed_mm_s)

    def test_signed_cross_track_and_heading_feedback_correct_radar_error(self) -> None:
        controller = PurePursuitController()
        path = NavigationPath(
            (PathPoint(0, 0, 0), PathPoint(100, 0, 0)),
            NavigationGoal(100, 0),
        )
        left_of_path = controller.compute(NavigationPose(10, 10, 0, 0), path)
        right_of_path = controller.compute(NavigationPose(10, -10, 0, 0), path)
        heading_left = controller.compute(NavigationPose(10, 0, 10, 0), path)

        self.assertGreater(left_of_path.signed_cross_track_error_cm, 0)
        self.assertLess(left_of_path.steering_angle_rad, 0)
        self.assertLess(right_of_path.signed_cross_track_error_cm, 0)
        self.assertGreater(right_of_path.steering_angle_rad, 0)
        self.assertLess(heading_left.heading_error_deg, 0)
        self.assertLess(heading_left.steering_angle_rad, 0)

    def test_tracking_error_reduces_speed(self) -> None:
        controller = PurePursuitController(
            config=PurePursuitConfig(approach_speed_mm_s=20.0)
        )
        path = NavigationPath(
            (PathPoint(0, 0, 0), PathPoint(200, 0, 0)),
            NavigationGoal(200, 0),
        )
        centered = controller.compute(NavigationPose(10, 0, 0, 0), path)
        displaced = controller.compute(NavigationPose(10, 15, 0, 0), path)
        self.assertLess(displaced.speed_mm_s, centered.speed_mm_s)

    def test_progress_window_does_not_jump_back_to_old_segment(self) -> None:
        controller = PurePursuitController()
        points = tuple(PathPoint(float(x), 0, 0) for x in range(0, 110, 10))
        path = NavigationPath(points, NavigationGoal(100, 0))
        command = controller.compute(
            NavigationPose(15, 0, 0, 0),
            path,
            min_path_index=5,
        )
        self.assertGreaterEqual(command.nearest_path_index, 5)

    def test_feedback_cannot_exceed_physical_turn_radius(self) -> None:
        from components.ackermann_drive import plan_ackermann_motion

        geometry = VehicleGeometry()
        controller = PurePursuitController(geometry)
        path = NavigationPath(
            (PathPoint(0, 0, 0), PathPoint(100, 0, 0)),
            NavigationGoal(100, 0),
        )
        cases = (
            NavigationPose(10, 50, 30, 0),
            NavigationPose(10, -50, 330, 0),
        )
        for pose in cases:
            command = controller.compute(pose, path)
            self.assertGreaterEqual(
                command.steering_angle_rad,
                geometry.min_right_steering_rad,
            )
            self.assertLessEqual(
                command.steering_angle_rad,
                geometry.max_left_steering_rad,
            )
            plan = plan_ackermann_motion(
                command.speed_mm_s,
                command.steering_angle_rad,
                wheelbase_mm=geometry.wheelbase_cm * 10.0,
                track_width_mm=geometry.track_width_cm * 10.0,
                firmware_track_width_mm=164.0,
                min_turn_radius_mm=geometry.min_turn_radius_cm * 10.0,
            )
            if plan.turn_radius_mm is not None:
                self.assertGreaterEqual(
                    abs(plan.turn_radius_mm) + 1e-9,
                    geometry.min_turn_radius_cm * 10.0,
                )

    def test_final_path_tangent_is_extended_for_stable_straight_approach(self) -> None:
        controller = PurePursuitController()
        path = NavigationPath(
            tuple(PathPoint(float(x), 0.0, 0.0) for x in range(0, 91, 10)),
            NavigationGoal(100.0, 0.0),
        )

        target = controller._lookahead_target(
            path,
            nearest_index=7,
            projection=(75.0, 0.0),
            direction=MotorDirection.FORWARD,
            lookahead_cm=60.0,
        )
        command = controller.compute(NavigationPose(75.0, 1.0, 356.0), path)

        self.assertEqual(target, (135.0, 0.0))
        self.assertEqual(
            controller._lookahead_target(
                path,
                nearest_index=8,
                projection=(90.0, 0.0),
                direction=MotorDirection.FORWARD,
                lookahead_cm=60.0,
            ),
            (150.0, 0.0),
        )
        self.assertLess(abs(command.steering_angle_rad), 0.12)


class _FakeDrive:
    def __init__(self) -> None:
        self.is_running = False
        self.commands: list[tuple[float, float, MotorDirection]] = []
        self.stop_count = 0

    def start(self):
        self.is_running = True
        return self

    def set_motion(self, speed, steering, *, direction, rear_differential_linked):
        self.commands.append((speed, steering, direction))

    def stop(self, *, center_steering=True):
        self.stop_count += 1

    def close(self):
        self.is_running = False


class NavigationStateMachineTests(unittest.TestCase):
    def wait_for(self, predicate, timeout=1.5) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.01)
        return False

    def test_stale_localization_stops_vehicle(self) -> None:
        drive = _FakeDrive()
        navigation = Navigation(
            drive=drive,
            config=NavigationConfig(control_hz=50, localization_timeout_s=0.05),
        )
        navigation.start()
        try:
            navigation.set_map(open_grid())
            navigation.update_pose(NavigationPose(0, 0, 0, time.monotonic() - 1))
            navigation.set_goal(NavigationGoal(100, 0))
            navigation.start_navigation()
            self.assertTrue(
                self.wait_for(lambda: navigation.state is NavigationState.LOCALIZATION_LOST)
            )
            self.assertGreater(drive.stop_count, 0)
            self.assertEqual(drive.commands, [])
        finally:
            navigation.close()

    def test_reaches_goal_and_stops_without_command(self) -> None:
        drive = _FakeDrive()
        navigation = Navigation(drive=drive, config=NavigationConfig(control_hz=50))
        navigation.start()
        try:
            navigation.set_map(open_grid())
            navigation.update_pose(NavigationPose(50, 20, 90))
            navigation.set_goal(NavigationGoal(50, 20, final_heading_deg=90))
            navigation.start_navigation()
            self.assertTrue(self.wait_for(lambda: navigation.state is NavigationState.ARRIVED))
            self.assertEqual(drive.commands, [])
            self.assertGreater(drive.stop_count, 0)
        finally:
            navigation.close()

    def test_cancel_clears_mission_but_retains_pose_and_map(self) -> None:
        drive = _FakeDrive()
        navigation = Navigation(drive=drive)
        grid = open_grid()
        pose = NavigationPose(42.0, -17.0, 25.0)
        navigation.set_map(grid)
        navigation.update_pose(pose)
        navigation.set_goal(NavigationGoal(100.0, 20.0))

        navigation.cancel(reason="ready for next goal; startup origin retained")

        self.assertEqual(navigation.pose, pose)
        self.assertIs(navigation._grid, grid)
        self.assertIs(navigation.state, NavigationState.IDLE)
        self.assertIn("origin retained", navigation.state_reason)

    def test_direction_change_stops_before_reverse(self) -> None:
        drive = _FakeDrive()
        drive.start()
        navigation = Navigation(
            drive=drive,
            config=NavigationConfig(gear_change_stop_s=0.25),
        )
        now = time.monotonic()
        reverse_path = NavigationPath(
            (
                PathPoint(0, 0, 0),
                PathPoint(-60, 0, 0, MotorDirection.REVERSE),
            ),
            NavigationGoal(-60, 0),
            map_revision=1,
        )
        navigation._active = True
        navigation._pose = NavigationPose(0, 0, 0, now)
        navigation._grid = open_grid()
        navigation._map_revision = 1
        navigation._goal = reverse_path.goal
        navigation._path = reverse_path
        navigation._last_direction = MotorDirection.FORWARD

        navigation._control_step(now)
        self.assertIs(navigation.state, NavigationState.GEAR_CHANGE)
        self.assertEqual(drive.commands, [])
        self.assertGreater(drive.stop_count, 0)

        navigation._control_step(now + 0.3)
        self.assertEqual(len(drive.commands), 1)
        self.assertIs(drive.commands[0][2], MotorDirection.REVERSE)

    def test_each_radar_pose_closes_lateral_steering_loop(self) -> None:
        drive = _FakeDrive()
        drive.start()
        navigation = Navigation(drive=drive, config=NavigationConfig(control_hz=20))
        now = time.monotonic()
        path = NavigationPath(
            (PathPoint(0, 0, 0), PathPoint(100, 0, 0)),
            NavigationGoal(100, 0),
            map_revision=1,
        )
        navigation._active = True
        navigation._grid = open_grid()
        navigation._map_revision = 1
        navigation._goal = path.goal
        navigation._path = path

        self.assertTrue(navigation.update_from_radar(self._radar_update(10, 10, 0, 2.0)))
        navigation._control_step(now)
        self.assertLess(drive.commands[-1][1], 0)

        self.assertTrue(navigation.update_from_radar(self._radar_update(10, -10, 0, 2.0)))
        navigation._control_step(now + 0.1)
        self.assertGreater(drive.commands[-1][1], 0)

    def test_bad_but_accepted_icp_slows_motion(self) -> None:
        drive = _FakeDrive()
        drive.start()
        navigation = Navigation(drive=drive, config=NavigationConfig())
        good = navigation._radar_speed_scale(self._radar_update(0, 0, 0, 2.0))
        poor = navigation._radar_speed_scale(self._radar_update(0, 0, 0, 10.0))
        self.assertEqual(good, 1.0)
        self.assertAlmostEqual(poor, navigation.config.minimum_radar_speed_scale)

    def test_radar_map_refresh_preserves_clear_path_and_invalidates_blocked_path(self) -> None:
        drive = _FakeDrive()
        navigation = Navigation(drive=drive)
        navigation.set_map(open_grid())
        path = NavigationPath(
            (PathPoint(0, 0, 0), PathPoint(100, 0, 0)),
            NavigationGoal(100, 0),
            map_revision=1,
        )
        navigation._path = path

        far_obstacle = OccupancyGrid.from_obstacle_points(
            [(0, 100)],
            resolution_cm=5.0,
            origin_x_cm=-150.0,
            origin_y_cm=-150.0,
            width=80,
            height=80,
        )
        navigation.set_map(far_obstacle)
        self.assertIsNotNone(navigation.path)
        self.assertEqual(navigation.path.map_revision, 2)

        blocked = OccupancyGrid.from_obstacle_points(
            [(50, 0)],
            resolution_cm=5.0,
            origin_x_cm=-150.0,
            origin_y_cm=-150.0,
            width=80,
            height=80,
        )
        navigation.set_map(blocked)
        self.assertIsNone(navigation.path)

    def test_deviation_must_be_confirmed_by_distinct_pose_updates(self) -> None:
        drive = _FakeDrive()
        drive.start()
        navigation = Navigation(
            drive=drive,
            config=NavigationConfig(deviation_replan_samples=3),
        )
        now = time.monotonic()
        path = NavigationPath(
            (PathPoint(0, 0, 0), PathPoint(100, 0, 0)),
            NavigationGoal(100, 0),
            map_revision=1,
        )
        navigation._active = True
        navigation._grid = open_grid()
        navigation._map_revision = 1
        navigation._goal = path.goal
        navigation._path = path
        navigation.update_pose(NavigationPose(10, 40, 0, now))

        navigation._control_step(now)
        navigation._control_step(now + 0.01)
        self.assertIsNotNone(navigation.path)
        self.assertEqual(navigation._deviation_count, 1)

        navigation.update_pose(NavigationPose(10, 40, 0, now + 0.02))
        navigation._control_step(now + 0.02)
        self.assertIsNotNone(navigation.path)
        navigation.update_pose(NavigationPose(10, 40, 0, now + 0.03))
        navigation._control_step(now + 0.03)
        self.assertIsNone(navigation.path)

    @staticmethod
    def _radar_update(
        x_cm: float,
        y_cm: float,
        yaw_cw_deg: float,
        mean_error_cm: float,
    ) -> RadarLocalizationUpdate:
        icp = ICPResult(Pose2D(), 80, mean_error_cm, 3)
        odometry = RadarOdometryUpdate(Pose2D(), True, True, icp)
        return RadarLocalizationUpdate(
            RadarScan((), 0, 0),
            odometry,
            Pose2D(x_cm, y_cm, yaw_cw_deg),
        )


if __name__ == "__main__":
    unittest.main()
