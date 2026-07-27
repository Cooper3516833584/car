from pathlib import Path
import threading
import time
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.fleet_models import AckStatus, DisasterRescueCommand, TerrainCode
from components.grid_rescue_mission import (
    AdjacentGridNavigator,
    AdjacentGridRescuePlanner,
    GridLayout,
    GridRescueMissionController,
    InPlaceDifferentialTurn,
    InPlaceTurnConfig,
    overlay_blocked_terrain,
)
from components.navigation import NavigationPose, OccupancyGrid


def command():
    terrain = (
        TerrainCode.FIELD, TerrainCode.LAKE, TerrainCode.FIELD, TerrainCode.FIELD, TerrainCode.FIELD,
        TerrainCode.FIELD, TerrainCode.SETTLEMENTS, TerrainCode.FIELD, TerrainCode.RIVER, TerrainCode.FIELD,
        TerrainCode.FIELD, TerrainCode.FIELD, TerrainCode.FIELD, TerrainCode.FIELD, TerrainCode.WILDFIRE,
    )
    return DisasterRescueCommand(7, 2, 4, tuple(int(value) for value in terrain))


class AdjacentGridPlannerTests(unittest.TestCase):
    def planner(self):
        return AdjacentGridRescuePlanner(
            water_terrain_codes=(TerrainCode.LAKE, TerrainCode.RIVER),
            wildfire_terrain_codes=(TerrainCode.WILDFIRE,),
            forbidden_terrain_codes=(TerrainCode.SETTLEMENTS,),
        )

    def test_route_is_adjacent_and_enters_water_exactly_once(self):
        plan = self.planner().plan(command())
        route = plan.driven_cells
        for first, second in zip(route, route[1:]):
            self.assertEqual(1, abs(first[0] - second[0]) + abs(first[1] - second[1]))
        water = {(0, 1), (1, 3)}
        self.assertEqual(1, sum(cell in water for cell in route))
        self.assertEqual((2, 4), plan.wildfire_cell)

    def test_nearest_reachable_water_wins_before_total_route_length(self):
        terrain = [int(TerrainCode.FIELD)] * 15
        terrain[1] = int(TerrainCode.LAKE)
        terrain[10] = int(TerrainCode.RIVER)
        terrain[14] = int(TerrainCode.WILDFIRE)

        plan = self.planner().plan(
            DisasterRescueCommand(8, 2, 4, tuple(terrain))
        )

        self.assertEqual((0, 1), plan.water_cell)

    def test_layout_uses_five_x_columns_and_three_y_rows(self):
        layout = GridLayout()
        self.assertEqual((-45.0, 125.0), layout.centre((0, 0)))
        self.assertEqual((25.0, 125.0), layout.centre((0, 1)))
        self.assertEqual((95.0, 195.0), layout.centre((1, 2)))
        self.assertEqual(
            ((0, 0), (0, 1), (0, 2), (0, 3), (0, 4)),
            layout.start_entry_cells,
        )
        self.assertEqual(0.0, layout.step_heading_deg((1, 1), (1, 2)))
        self.assertEqual(90.0, layout.step_heading_deg((1, 1), (2, 1)))
        self.assertEqual(180.0, layout.step_heading_deg((1, 1), (1, 0)))
        self.assertEqual(270.0, layout.step_heading_deg((1, 1), (0, 1)))

    def test_layout_rejects_diagonal_or_skipped_step(self):
        layout = GridLayout()
        for target in ((2, 2), (1, 3), (1, 1)):
            with self.subTest(target=target), self.assertRaises(ValueError):
                layout.step_heading_deg((1, 1), target)

    def test_requested_commissioning_route_is_adjacent_and_avoids_water_after_pickup(self):
        terrain = [int(TerrainCode.FIELD)] * 15
        terrain[1] = int(TerrainCode.RIVER)
        terrain[7] = int(TerrainCode.WILDFIRE)
        plan = self.planner().plan(
            DisasterRescueCommand(1, 1, 2, tuple(terrain))
        )
        self.assertEqual((0, 1), plan.water_cell)
        self.assertEqual((1, 2), plan.wildfire_cell)
        self.assertNotIn((0, 1), plan.to_wildfire)
        self.assertNotIn((0, 1), plan.to_start_entry)
        self.assertNotIn((0, 1), plan.blocked_to_water)
        self.assertIn((0, 1), plan.blocked_after_water)
        self.assertNotIn((0, 2), plan.blocked_after_water)
        route = plan.driven_cells
        for first, second in zip(route, route[1:]):
            self.assertEqual(
                1,
                abs(first[0] - second[0]) + abs(first[1] - second[1]),
            )

    def test_latest_survey_can_start_outside_grid_and_return_without_second_water(self):
        terrain = (
            TerrainCode.RIVER, TerrainCode.LAKE, TerrainCode.SNOW_MOUNTAIN,
            TerrainCode.SNOW_MOUNTAIN, TerrainCode.RIVER,
            TerrainCode.FIELD, TerrainCode.SETTLEMENTS, TerrainCode.WILDFIRE,
            TerrainCode.LAKE, TerrainCode.FIELD,
            TerrainCode.SNOW_MOUNTAIN, TerrainCode.DEBRIS_FLOW,
            TerrainCode.SETTLEMENTS, TerrainCode.SETTLEMENTS, TerrainCode.FIELD,
        )
        plan = self.planner().plan(
            DisasterRescueCommand(9, 1, 2, tuple(int(value) for value in terrain))
        )
        actual_water = {(0, 0), (0, 1), (0, 4), (1, 3)}
        self.assertIn(plan.water_cell, actual_water)
        self.assertEqual((1, 2), plan.wildfire_cell)
        self.assertEqual(1, sum(cell in actual_water for cell in plan.driven_cells))
        self.assertIn(plan.to_start_entry[-1], GridLayout().start_entry_cells)

    def test_configured_field_can_be_used_as_water(self):
        planner = AdjacentGridRescuePlanner(
            water_terrain_codes=(TerrainCode.FIELD,),
            wildfire_terrain_codes=(TerrainCode.WILDFIRE,),
            forbidden_terrain_codes=(),
        )
        self.assertNotIn(planner.plan(command()).water_cell, {(0, 1), (1, 3)})

    def test_overlay_only_adds_obstacles(self):
        cells = [0] * (60 * 60)
        cells[1] = 100
        grid = OccupancyGrid(10.0, -100.0, -400.0, 60, 60, tuple(cells), unknown_is_occupied=False)
        overlaid = overlay_blocked_terrain(grid, {(0, 1)})
        self.assertEqual(100, overlaid.cells[1])
        ix, iy = overlaid.world_to_cell(*GridLayout().centre((0, 1)))
        self.assertTrue(overlaid.is_occupied(ix, iy))


class ControllerTests(unittest.TestCase):
    def test_three_second_stages_are_sequenced_without_rx_thread_work(self):
        planner = AdjacentGridPlannerTests().planner()
        calls = []
        result_event = threading.Event()
        results = []
        controller = GridRescueMissionController(
            planner,
            navigate=lambda cell: calls.append(("navigate", cell)) or True,
            set_step_overlay=lambda current, target, blocked: calls.append(
                ("overlay", current, target, blocked)
            ),
            clear_overlay=lambda: calls.append(("clear",)),
            indicator=lambda stage, active: calls.append((stage, active)),
            on_result=lambda result: (results.append(result), result_event.set()),
            hold_seconds=0.0,
        )
        self.assertEqual(AckStatus.ACCEPTED, controller.submit(command()).status)
        self.assertTrue(result_event.wait(1.0))
        self.assertEqual(AckStatus.COMPLETED, results[-1].status)
        self.assertIn(("water", True), calls)
        self.assertIn(("wildfire", True), calls)
        self.assertEqual(("navigate", None), [call for call in calls if call[0] == "navigate"][-1])

    def test_controller_can_use_adjacent_motion_component(self):
        planner = AdjacentGridPlannerTests().planner()
        moves = []
        result_event = threading.Event()
        controller = GridRescueMissionController(
            planner,
            navigate=lambda _cell: self.fail("legacy navigate callback used"),
            move_adjacent=lambda current, target: moves.append((current, target)) or True,
            set_step_overlay=lambda *_args: None,
            clear_overlay=lambda: None,
            on_result=lambda _result: result_event.set(),
            hold_seconds=0.0,
        )
        self.assertEqual(AckStatus.ACCEPTED, controller.submit(command()).status)
        self.assertTrue(result_event.wait(1.0))
        self.assertEqual((None, (0, 1)), moves[0])
        self.assertIn(moves[-1][0], GridLayout().start_entry_cells)
        self.assertIsNone(moves[-1][1])

    def test_stop_interrupts_three_second_hold(self):
        planner = AdjacentGridPlannerTests().planner()
        water_hold_started = threading.Event()
        result_event = threading.Event()
        results = []
        controller = GridRescueMissionController(
            planner,
            navigate=lambda _cell: True,
            set_step_overlay=lambda *_args: None,
            clear_overlay=lambda: None,
            indicator=lambda stage, active: (
                water_hold_started.set()
                if stage == "water" and active
                else None
            ),
            on_result=lambda result: (results.append(result), result_event.set()),
        )
        self.assertEqual(AckStatus.ACCEPTED, controller.submit(command()).status)
        self.assertTrue(water_hold_started.wait(1.0))

        controller.stop()

        self.assertTrue(result_event.wait(1.0))
        self.assertEqual(AckStatus.FAILED, results[-1].status)
        self.assertTrue(controller.wait(0.1))


class _FakeRearMotors:
    allow_in_place_rotation = True

    def __init__(self):
        self.commands = []
        self.stops = 0

    def set_wheels(self, left, right):
        self.commands.append((left, right))

    def stop(self):
        self.stops += 1


class _FakeSteering:
    def __init__(self):
        self.centres = 0

    def center(self):
        self.centres += 1


class _FakeDrive:
    def __init__(self):
        self.rear_motors = _FakeRearMotors()
        self.steering = _FakeSteering()
        self.stop_calls = []

    def stop(self, *, center_steering=True):
        self.stop_calls.append(center_steering)
        self.rear_motors.stop()
        if center_steering:
            self.steering.center()


class AdjacentGridMotionTests(unittest.TestCase):
    def test_pivot_centres_front_and_uses_opposite_rear_speeds(self):
        drive = _FakeDrive()
        now = time.monotonic()
        poses = iter(
            (
                NavigationPose(0, 0, 0, now),
                NavigationPose(0, 0, 88, now + 0.01),
                NavigationPose(0, 0, 90, now + 0.02),
            )
        )
        latest = [NavigationPose(0, 0, 90, now + 0.02)]

        def pose_provider():
            try:
                latest[0] = next(poses)
            except StopIteration:
                pass
            return latest[0]

        motion = []
        pivot = InPlaceDifferentialTurn(
            drive,  # type: ignore[arg-type]
            pose_provider=pose_provider,
            config=InPlaceTurnConfig(
                refresh_interval_s=0.001,
                timeout_s=0.2,
            ),
            on_motion_changed=motion.append,
        )

        pivot.turn_to(90)

        self.assertEqual([True], drive.stop_calls)
        self.assertIn((-80.0, 80.0), drive.rear_motors.commands)
        self.assertGreaterEqual(drive.steering.centres, 2)
        self.assertEqual([True, False], motion)

    def test_adjacent_navigator_pivots_then_targets_exact_cell_centre(self):
        turns = []
        goals = []

        class Pivot:
            def turn_to(self, heading):
                turns.append(heading)

        navigator = AdjacentGridNavigator(
            Pivot(),  # type: ignore[arg-type]
            navigate_to=lambda x, y, heading: goals.append((x, y, heading)) or True,
        )

        self.assertTrue(navigator.move((1, 1), (1, 2)))
        self.assertEqual([0.0], turns)
        self.assertEqual([(95.0, 195.0, 0.0)], goals)

        with self.assertRaises(ValueError):
            navigator.move((1, 1), (2, 2))

    def test_return_to_start_does_not_constrain_final_heading(self):
        turns = []
        goals = []

        class Pivot:
            def turn_to(self, heading):
                turns.append(heading)

        navigator = AdjacentGridNavigator(
            Pivot(),  # type: ignore[arg-type]
            navigate_to=lambda x, y, heading: goals.append((x, y, heading)) or True,
        )

        self.assertTrue(navigator.move((0, 0), None))
        self.assertEqual([], turns)
        self.assertEqual([(0.0, 0.0, None)], goals)


if __name__ == "__main__":
    unittest.main()
