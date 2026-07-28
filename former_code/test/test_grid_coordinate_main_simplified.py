"""Hardware-free checks for adjacent-grid rescue wiring in production main."""

from pathlib import Path
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components import DroneGlobalAlignment
from components.fleet_models import AckReason, AckStatus, DisasterRescueCommand, TerrainCode
from main import (
    CarMainApplication,
    FORBIDDEN_TERRAINS,
    MainConfig,
    WATER_PICKUP_TERRAINS,
    WILDFIRE_TARGET_TERRAINS,
    terrain_codes,
)


class ProductionMainGridRescueTests(unittest.TestCase):
    def make_app(self):
        return CarMainApplication(
            MainConfig(console_enabled=False),
            hmac_key=None,
            fleet_bus=True,
        )

    def test_production_main_registers_disaster_handler(self):
        app = self.make_app()

        self.assertIsNotNone(app.rescue_controller)
        self.assertIsNotNone(app.fleet_node)
        self.assertIsNotNone(app.fleet_node._on_disaster_rescue)
        self.assertTrue(
            app.coordinate_navigation.config.allow_in_place_rotation
        )

    def test_disaster_command_is_rejected_until_mapping_and_frame_sync(self):
        app = self.make_app()
        terrain = [int(TerrainCode.FIELD)] * 15
        terrain[1] = int(TerrainCode.LAKE)
        terrain[7] = int(TerrainCode.WILDFIRE)

        result = app._fleet_disaster_rescue(
            DisasterRescueCommand(1, 1, 2, tuple(terrain))
        )

        self.assertEqual(AckStatus.REJECTED, result.status)
        self.assertEqual(AckReason.NOT_READY, result.reason)

    def test_production_handler_runs_the_adjacent_plan_to_completion(self):
        app = self.make_app()
        app._ready = True
        app._fleet_alignment = DroneGlobalAlignment(0.0, 0.0, 0.0)
        app.trusted_map._calibration = object()
        moves = []
        result_event = threading.Event()
        results = []
        controller = app.rescue_controller
        assert controller is not None
        original_on_result = controller._on_result
        controller._move_adjacent = (
            lambda current, target: moves.append((current, target)) or True
        )
        controller._set_step_overlay = lambda *_args: None
        controller._clear_overlay = lambda: None
        controller._hold_seconds = 0.0
        controller._on_result = lambda result: (
            results.append(result),
            original_on_result(result),
            result_event.set(),
        )
        terrain = [int(TerrainCode.FIELD)] * 15
        terrain[1] = int(TerrainCode.LAKE)
        terrain[7] = int(TerrainCode.WILDFIRE)

        accepted = app._fleet_disaster_rescue(
            DisasterRescueCommand(2, 1, 2, tuple(terrain))
        )

        self.assertEqual(AckStatus.ACCEPTED, accepted.status)
        self.assertTrue(result_event.wait(1.0))
        self.assertEqual(AckStatus.COMPLETED, results[-1].status)
        self.assertEqual((None, (0, 0)), moves[0])
        self.assertEqual(((0, 0), None), moves[-1])
        self.assertFalse(app._rescue_mission_active)

    def test_three_operator_lists_are_valid_and_editable(self):
        self.assertEqual(
            (int(TerrainCode.LAKE), int(TerrainCode.RIVER)),
            terrain_codes(WATER_PICKUP_TERRAINS),
        )
        self.assertEqual(
            (int(TerrainCode.WILDFIRE),),
            terrain_codes(WILDFIRE_TARGET_TERRAINS),
        )
        self.assertEqual((), terrain_codes(FORBIDDEN_TERRAINS))
        self.assertEqual(
            (int(TerrainCode.FIELD),),
            terrain_codes([" FIELD "]),
        )
        with self.assertRaises(ValueError):
            terrain_codes(["not-a-terrain"])


if __name__ == "__main__":
    unittest.main()
