from pathlib import Path
import threading
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.fleet_models import AckStatus, DisasterRescueCommand, TerrainCode
from components.grid_rescue_mission import (
    AdjacentGridRescuePlanner,
    GridLayout,
    GridRescueMissionController,
    overlay_blocked_terrain,
)
from components.navigation import OccupancyGrid


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

    def test_layout_uses_five_x_columns_and_three_y_rows(self):
        layout = GridLayout()
        self.assertEqual((0.0, 0.0), layout.centre((0, 0)))
        self.assertEqual((70.0, 0.0), layout.centre((0, 1)))
        self.assertEqual((140.0, 70.0), layout.centre((1, 2)))

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


if __name__ == "__main__":
    unittest.main()
