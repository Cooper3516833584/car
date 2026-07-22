from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_ros_bridge.ros_conversions import (  # noqa: E402
    Quaternion, quaternion_to_yaw, radar_pose_to_ros, radar_points_to_scan_ranges, yaw_to_quaternion,
)
from components.radar_driver import Pose2D, RadarPoint  # noqa: E402


class RosConversionsTests(unittest.TestCase):
    def test_cw_radar_yaw_becomes_ccw_ros_yaw(self) -> None:
        self.assertAlmostEqual(radar_pose_to_ros(Pose2D(100, 50, 90)).yaw_rad, -1.57079632679)

    def test_quaternion_round_trip(self) -> None:
        self.assertAlmostEqual(quaternion_to_yaw(yaw_to_quaternion(1.2)), 1.2)

    def test_clockwise_radar_bin_is_ros_right_side(self) -> None:
        ranges = radar_points_to_scan_ranges([RadarPoint(90.0, 1000, 20)], bins=360)
        self.assertEqual(ranges[90], 1.0)


if __name__ == "__main__":
    unittest.main()
