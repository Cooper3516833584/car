from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_ros_bridge.field_geometry import FieldGeometry  # noqa: E402
from car_ros_bridge.mission_bridge_node import MissionBridgeLogic  # noqa: E402
from components import NavigationCommandReceipt, NavigationGoal, NavigationCommandRejected  # noqa: E402


class MissionBridgeLogicTests(unittest.TestCase):
    def setUp(self) -> None:
        self.logic = MissionBridgeLogic(FieldGeometry(((0, 0), (2, 0), (2, 2), (0, 2))))

    def test_requires_localization_and_single_active_goal(self) -> None:
        receipt = NavigationCommandReceipt(1, 2, 0x20)
        with self.assertRaises(NavigationCommandRejected): self.logic.submit(NavigationGoal(100, 100), receipt)
        self.logic.localization_ready = True; self.logic.submit(NavigationGoal(100, 100), receipt)
        with self.assertRaises(NavigationCommandRejected): self.logic.submit(NavigationGoal(150, 100), receipt)


if __name__ == "__main__": unittest.main()
