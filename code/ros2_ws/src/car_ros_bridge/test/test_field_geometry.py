from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_ros_bridge.field_geometry import FieldGeometry  # noqa: E402


class FieldGeometryTests(unittest.TestCase):
    def test_goal_stays_inside_measured_boundary_and_outside_obstacle(self) -> None:
        field = FieldGeometry(((0, 0), (4, 0), (4, 2), (0, 2)), (((1, 0.5), (2, 0.5), (2, 1.5), (1, 1.5)),))
        self.assertTrue(field.contains_goal(3, 1))
        self.assertFalse(field.contains_goal(1.5, 1))
        self.assertFalse(field.contains_goal(5, 1))


if __name__ == "__main__":
    unittest.main()
