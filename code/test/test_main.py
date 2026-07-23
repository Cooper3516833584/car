"""Hardware-free tests for the production startup/command coordinator."""

from __future__ import annotations

import logging
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components import (  # noqa: E402
    AckStatus,
    CoordinateFrameTransform,
    DroneGlobalAlignment,
    ICPResult,
    NavigationCommandReceipt,
    NavigationCommandRejected,
    NavigationGoal,
    NavigationState,
    Pose2D,
    RadarLocalizationUpdate,
    RadarScan,
    RectangularWallReference,
    RectangleFieldCalibration,
    WallFusionResult,
    pack_navigation_command,
    unpack_authenticated_frame,
)
from components.radar_driver import RadarOdometryUpdate  # noqa: E402
from main import (  # noqa: E402
    CarMainApplication,
    MainConfig,
    NAVIGATION_ALLOW_REVERSE,
    NAVIGATION_CRUISE_SPEED_CM_S,
    NAVIGATION_REVERSE_SPEED_CM_S,
    build_argument_parser,
    configure_logging,
    default_log_dir,
    parse_console_command,
    rebase_calibration_to_start_pose,
    shutdown_logging,
)


KEY = bytes.fromhex("00112233445566778899aabbccddeeff")


def make_calibration() -> RectangleFieldCalibration:
    identity = DroneGlobalAlignment(0.0, 0.0, 0.0)
    return RectangleFieldCalibration(
        identity,
        RectangularWallReference(identity, -100.0, -50.0),
        Pose2D(),
        -100.0,
        200.0,
        -50.0,
        150.0,
        0.0,
        4,
    )


class FakeNavigation:
    def __init__(self) -> None:
        self.goal: NavigationGoal | None = None
        self.started = False
        self.cancel_reasons: list[str] = []

    def set_goal(self, goal: NavigationGoal) -> None:
        self.goal = goal

    def start_navigation(self) -> None:
        self.started = True

    def cancel(self, *, reason: str = "navigation cancelled") -> None:
        self.cancel_reasons.append(reason)
        self.goal = None
        self.started = False


class ImmediateArrivalNavigation(FakeNavigation):
    def __init__(self, app: CarMainApplication) -> None:
        super().__init__()
        self.app = app

    def start_navigation(self) -> None:
        super().start_navigation()
        self.app._on_navigation_state(NavigationState.ARRIVED, "already there")


class FakeLink:
    def __init__(self) -> None:
        self.frames: list[bytes] = []

    def write(self, frame: bytes) -> None:
        self.frames.append(frame)


class MainCoordinatorTests(unittest.TestCase):
    def make_app(self) -> CarMainApplication:
        app = CarMainApplication(MainConfig(), hmac_key=KEY)
        app._calibration = make_calibration()
        app._ready = True
        return app

    @staticmethod
    def make_radar_update(
        pose: Pose2D,
        *,
        error_cm: float = 1.0,
        points: tuple[tuple[float, float], ...] = (),
        wall_fusion: WallFusionResult | None = None,
    ) -> RadarLocalizationUpdate:
        icp = ICPResult(Pose2D(), 100, error_cm, 3)
        return RadarLocalizationUpdate(
            RadarScan((), 1000, 3600),
            RadarOdometryUpdate(pose, True, True, icp),
            pose,
            points,
            wall_fusion,
        )

    def test_grid_marks_field_exterior_occupied(self) -> None:
        app = self.make_app()
        calibration = make_calibration()
        grid = app._build_grid([(25.0, 25.0)], calibration)

        outside = grid.world_to_cell(-110.0, 0.0)
        inside = grid.world_to_cell(0.0, 0.0)
        radar_hit = grid.world_to_cell(25.0, 25.0)
        self.assertTrue(grid.is_occupied(*outside))
        self.assertFalse(grid.is_occupied(*inside))
        self.assertTrue(grid.is_occupied(*radar_hit))

    def test_trusted_localization_rejects_pose_jump_and_field_exit(self) -> None:
        app = self.make_app()
        app._last_trusted_pose = Pose2D()

        jump = app._trusted_localization_rejection(
            self.make_radar_update(Pose2D(30.0, 0.0, 0.0))
        )
        outside = app._trusted_localization_rejection(
            self.make_radar_update(Pose2D(195.0, 0.0, 0.0))
        )

        self.assertIn("translation jump", jump or "")
        self.assertIn("footprint outside", outside or "")

    def test_trusted_localization_rejects_large_icp_error(self) -> None:
        app = self.make_app()
        app._last_trusted_pose = Pose2D()

        rejection = app._trusted_localization_rejection(
            self.make_radar_update(Pose2D(2.0, 0.0, 0.0), error_cm=10.1)
        )

        self.assertIn("ICP error", rejection or "")

    def test_rejected_pose_does_not_update_navigation_or_trusted_map(self) -> None:
        app = self.make_app()
        app._last_trusted_pose = Pose2D()
        app._last_map_update = time.monotonic()
        update = self.make_radar_update(
            Pose2D(30.0, 0.0, 0.0),
            points=((50.0, 20.0),),
        )

        app._on_radar_update(update)

        self.assertIsNone(app.navigation.pose)
        self.assertEqual(app._trusted_map.cells(), [])
        self.assertIn("translation jump", app._last_trusted_rejection or "")

    def test_vehicle_footprint_points_are_removed_from_map(self) -> None:
        app = self.make_app()
        pose = Pose2D(50.0, 20.0, 0.0)

        retained = app._filter_vehicle_footprint_points(
            [(50.0, 20.0), (55.0, 20.0), (80.0, 20.0), (50.0, 40.0)],
            pose,
        )

        self.assertEqual(retained, [(80.0, 20.0), (50.0, 40.0)])

    def test_forced_grid_refresh_clears_historical_hits_under_current_car(self) -> None:
        app = self.make_app()
        pose = Pose2D(50.0, 20.0, 0.0)
        app._last_trusted_pose = pose
        app._trusted_map.add_points([(50.0, 20.0), (50.0, 20.0), (80.0, 20.0), (80.0, 20.0)])

        app._refresh_trusted_grid(force=True)

        grid = app._grid
        self.assertIsNotNone(grid)
        assert grid is not None
        self.assertFalse(grid.is_occupied(*grid.world_to_cell(50.0, 20.0)))
        self.assertTrue(grid.is_occupied(*grid.world_to_cell(80.0, 20.0)))

    def test_rejected_wall_correction_scan_does_not_pollute_trusted_map(self) -> None:
        app = self.make_app()
        app._last_trusted_pose = Pose2D()
        app._last_map_update = time.monotonic()
        pose = Pose2D(2.0, 0.0, 0.0)
        wall = WallFusionResult(True, False, None, pose, "wall X residual gate")

        app._on_radar_update(
            self.make_radar_update(
                pose,
                points=((50.0, 20.0),),
                wall_fusion=wall,
            )
        )

        self.assertEqual(app.navigation.pose.x_cm if app.navigation.pose else None, 2.0)
        self.assertEqual(app._trusted_map.cells(), [])

    def test_default_and_detailed_log_file(self) -> None:
        previous = os.environ.pop("CAR_LOG_DIR", None)
        try:
            self.assertEqual(
                default_log_dir(),
                Path(__file__).resolve().parents[1] / "logs",
            )
        finally:
            if previous is not None:
                os.environ["CAR_LOG_DIR"] = previous
        with tempfile.TemporaryDirectory() as directory:
            log_path = configure_logging(directory, "WARNING")
            logging.getLogger("car-main").debug("detailed-log-probe")
            shutdown_logging()
            self.assertEqual(log_path.name, "car-main.log")
            self.assertIn("detailed-log-probe", log_path.read_text(encoding="utf-8"))

    def test_top_level_drive_speeds_are_applied_to_navigation(self) -> None:
        app = self.make_app()
        self.assertEqual(
            app.navigation.controller.config.cruise_speed_mm_s,
            NAVIGATION_CRUISE_SPEED_CM_S * 10.0,
        )
        self.assertEqual(
            app.navigation.controller.config.reverse_speed_mm_s,
            NAVIGATION_REVERSE_SPEED_CM_S * 10.0,
        )
        self.assertEqual(
            app.navigation.drive.rear_motors.max_wheel_speed_mm_s,
            max(
                NAVIGATION_CRUISE_SPEED_CM_S,
                NAVIGATION_REVERSE_SPEED_CM_S,
            )
            * 10.0
            * 1.20,
        )

    def test_top_level_reverse_switch_and_cli_override(self) -> None:
        parser = build_argument_parser()
        self.assertTrue(NAVIGATION_ALLOW_REVERSE)
        self.assertTrue(MainConfig().allow_reverse)
        self.assertTrue(parser.parse_args([]).allow_reverse)
        self.assertFalse(parser.parse_args(["--no-reverse"]).allow_reverse)
        self.assertTrue(parser.parse_args(["--allow-reverse"]).allow_reverse)

    def test_start_frame_rebase_sets_position_and_heading_to_zero(self) -> None:
        edge_frame = DroneGlobalAlignment(0.0, 0.0, 20.0)
        identity = DroneGlobalAlignment(0.0, 0.0, 0.0)
        calibration = RectangleFieldCalibration(
            edge_frame,
            RectangularWallReference(identity, -100.0, -50.0),
            Pose2D(0.0, 0.0, 20.0),
            -100.0,
            200.0,
            -50.0,
            150.0,
            20.0,
            4,
        )

        rebased = rebase_calibration_to_start_pose(calibration)

        self.assertEqual(rebased.initial_global_pose, Pose2D())
        self.assertEqual(rebased.local_to_global.pose_to_global(Pose2D()), Pose2D())
        self.assertAlmostEqual(rebased.wall_reference.wall_to_global.yaw_offset_cw_deg, -20.0)
        self.assertTrue(rebased.contains_point(0.0, 0.0))

    def test_rotated_field_bbox_corners_remain_forbidden(self) -> None:
        edge_frame = DroneGlobalAlignment(0.0, 0.0, 20.0)
        identity = DroneGlobalAlignment(0.0, 0.0, 0.0)
        calibration = rebase_calibration_to_start_pose(
            RectangleFieldCalibration(
                edge_frame,
                RectangularWallReference(identity, -100.0, -50.0),
                Pose2D(0.0, 0.0, 20.0),
                -100.0,
                200.0,
                -50.0,
                150.0,
                20.0,
                4,
            )
        )
        app = self.make_app()
        grid = app._build_grid([], calibration)
        outside_polygon = grid.world_to_cell(
            calibration.min_x_cm + 1.0,
            calibration.min_y_cm + 1.0,
        )
        self.assertTrue(grid.is_occupied(*outside_polygon))
        self.assertFalse(grid.is_occupied(*grid.world_to_cell(0.0, 0.0)))

    def test_coordinate_and_optional_heading_are_forwarded(self) -> None:
        app = self.make_app()
        fake_navigation = FakeNavigation()
        app.navigation = fake_navigation  # type: ignore[assignment]
        receipt = NavigationCommandReceipt(10, 11, 0x20)

        app._on_goal_command(NavigationGoal(120.0, 80.0, 275.5), receipt)

        self.assertEqual(fake_navigation.goal, NavigationGoal(120.0, 80.0, 275.5))
        self.assertTrue(fake_navigation.started)
        self.assertEqual(app._active_receipt, receipt)

    def test_serial_coordinate_frame_moves_pose_map_and_field_together(self) -> None:
        app = self.make_app()
        app.radar.set_alignment(DroneGlobalAlignment(0.0, 0.0, 0.0))
        app._last_trusted_pose = Pose2D(10.0, 0.0, 0.0)
        app._trusted_map.add_points([(20.0, 30.0), (20.0, 30.0)])
        app.radar.global_map.add_points([(20.0, 30.0)])

        app._on_coordinate_frame_command(
            CoordinateFrameTransform(100.0, 200.0, 90.0),
            NavigationCommandReceipt(7, 8, 0x21),
        )

        self.assertTrue(app._coordinate_frame_synchronized)
        self.assertAlmostEqual(app._last_trusted_pose.x_cm, 100.0)
        self.assertAlmostEqual(app._last_trusted_pose.y_cm, 210.0)
        self.assertAlmostEqual(app._last_trusted_pose.yaw_cw_deg, -90.0)
        self.assertIsNotNone(app.navigation.pose)
        assert app.navigation.pose is not None
        self.assertAlmostEqual(app.navigation.pose.heading_deg, 90.0)
        self.assertTrue(app._calibration.contains_point(100.0, 200.0))
        transformed_cells = app._trusted_map.cells(min_hits=2)
        self.assertEqual(len(transformed_cells), 1)
        self.assertAlmostEqual(transformed_cells[0].x_cm, 70.0)
        self.assertAlmostEqual(transformed_cells[0].y_cm, 220.0)
        with self.assertRaises(NavigationCommandRejected):
            app._on_coordinate_frame_command(
                CoordinateFrameTransform(0.0, 0.0, 0.0),
                NavigationCommandReceipt(7, 9, 0x21),
            )

    def test_console_coordinate_and_optional_integer_heading(self) -> None:
        without_heading = parse_console_command("120 80")
        with_heading = parse_console_command("120, 80, 275")
        self.assertEqual(without_heading.goal, NavigationGoal(120.0, 80.0))
        self.assertEqual(with_heading.goal, NavigationGoal(120.0, 80.0, 275))

    def test_console_rejects_non_integer_or_out_of_range_heading(self) -> None:
        for command in ("10 20 90.0", "10 20 -1", "10 20 360"):
            with self.subTest(command=command), self.assertRaises(ValueError):
                parse_console_command(command)

    def test_console_goal_is_forwarded_to_navigation(self) -> None:
        app = self.make_app()
        fake_navigation = FakeNavigation()
        app.navigation = fake_navigation  # type: ignore[assignment]
        goal = NavigationGoal(120.0, 80.0, 90)

        app._submit_console_goal(goal)

        self.assertEqual(fake_navigation.goal, goal)
        self.assertTrue(fake_navigation.started)
        self.assertTrue(app._console_mission_active)

    def test_arrival_accepts_next_goal_without_changing_startup_origin(self) -> None:
        app = self.make_app()
        calibration = app._calibration
        fake_navigation = FakeNavigation()
        app.navigation = fake_navigation  # type: ignore[assignment]

        app._submit_console_goal(NavigationGoal(50.0, 20.0))
        app._on_navigation_state(NavigationState.ARRIVED, "goal reached")

        self.assertFalse(app._console_mission_active)
        self.assertIs(app._calibration, calibration)
        self.assertIn("origin retained", fake_navigation.cancel_reasons[-1])

        next_goal = NavigationGoal(120.0, 80.0, 90.0)
        app._submit_console_goal(next_goal)
        self.assertEqual(fake_navigation.goal, next_goal)
        self.assertTrue(fake_navigation.started)
        self.assertTrue(app._console_mission_active)

    def test_goal_outside_fitted_field_is_rejected(self) -> None:
        app = self.make_app()
        app.navigation = FakeNavigation()  # type: ignore[assignment]

        with self.assertRaises(NavigationCommandRejected):
            app._on_goal_command(
                NavigationGoal(201.0, 0.0),
                NavigationCommandReceipt(1, 2, 0x20),
            )

    def test_immediate_completion_ack_follows_acceptance(self) -> None:
        app = self.make_app()
        app.navigation = ImmediateArrivalNavigation(app)  # type: ignore[assignment]
        fake_link = FakeLink()
        app.link = fake_link  # type: ignore[assignment]

        app._on_link_frame(
            pack_navigation_command(
                NavigationGoal(0.0, 0.0),
                session=20,
                seq=21,
                key=KEY,
            )
        )

        statuses = [
            AckStatus(unpack_authenticated_frame(frame, key=KEY).payload[-2])
            for frame in fake_link.frames
        ]
        self.assertEqual(
            statuses,
            [AckStatus.RECEIVED, AckStatus.ACCEPTED, AckStatus.COMPLETED],
        )


if __name__ == "__main__":
    unittest.main()
