from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.radar_driver import (  # noqa: E402
    D500RadarComponent,
    DroneGlobalAlignment,
    GlobalCorrectionMode,
    Pose2D,
    RadarOdometryUpdate,
    RadarPacket,
    RadarScan,
    WallFusionConfig,
    WallFusionResult,
    WallPoseObservation,
)


class _OneScanAssembler:
    def __init__(self, scan: RadarScan) -> None:
        self.scan = scan

    def feed(self, _packet: RadarPacket):
        return [self.scan]


class _FixedOdometry:
    def __init__(self, pose: Pose2D) -> None:
        self.pose = pose

    def update(self, _scan: RadarScan) -> RadarOdometryUpdate:
        return RadarOdometryUpdate(self.pose, True, True)


class _WallLocalizer:
    reference = object()

    def observe(self, _scan: RadarScan, _predicted_global_pose: Pose2D):
        return WallPoseObservation(10.0, 5.0, 2.0)


def _packet() -> RadarPacket:
    return RadarPacket(0, 0.0, 0.0, 0, ())


def _scan() -> RadarScan:
    return RadarScan((), 0, 0)


class GlobalCorrectionModeTests(unittest.TestCase):
    def assert_pose_equal(self, actual: Pose2D, expected: Pose2D) -> None:
        self.assertAlmostEqual(actual.x_cm, expected.x_cm)
        self.assertAlmostEqual(actual.y_cm, expected.y_cm)
        self.assertAlmostEqual(actual.yaw_cw_deg, expected.yaw_cw_deg)

    def _radar(self, mode: GlobalCorrectionMode) -> D500RadarComponent:
        raw_pose = Pose2D(10.0, 5.0, 2.0)
        return D500RadarComponent(
            assembler=_OneScanAssembler(_scan()),
            odometry=_FixedOdometry(raw_pose),
            alignment=DroneGlobalAlignment.from_reference(Pose2D(), Pose2D(100.0, 0.0, 0.0)),
            wall_localizer=_WallLocalizer(),
            wall_fusion_config=WallFusionConfig(
                update_every_scans=1,
                consistency_samples=1,
            ),
            global_correction_mode=mode,
        )

    def test_legacy_mode_rewrites_local_odometry(self) -> None:
        radar = self._radar(GlobalCorrectionMode.LEGACY_REWRITE_ODOMETRY)
        fused = Pose2D(130.0, 0.0, 0.0)
        with patch(
            "components.radar_driver.fuse_wall_observation",
            return_value=WallFusionResult(True, True, None, fused),
        ):
            update = radar.process_packet(_packet())[0]
        self.assertEqual(update.odometry.pose, Pose2D(30.0, 0.0, 0.0))
        self.assertEqual(radar.odometry.pose, Pose2D(30.0, 0.0, 0.0))

    def test_alignment_mode_keeps_raw_odom_and_absorbs_correction(self) -> None:
        radar = self._radar(GlobalCorrectionMode.UPDATE_ALIGNMENT)
        raw_pose = radar.odometry.pose
        fused = Pose2D(130.0, -7.0, 8.0)
        with patch(
            "components.radar_driver.fuse_wall_observation",
            return_value=WallFusionResult(True, True, None, fused),
        ):
            update = radar.process_packet(_packet())[0]
        self.assertEqual(update.odometry.pose, raw_pose)
        self.assertEqual(radar.odometry.pose, raw_pose)
        alignment = radar.get_alignment()
        self.assertIsNotNone(alignment)
        self.assert_pose_equal(update.global_pose, fused)
        self.assert_pose_equal(alignment.pose_to_global(raw_pose), fused)

    def test_alignment_map_odom_base_composition_is_consistent(self) -> None:
        radar = self._radar(GlobalCorrectionMode.UPDATE_ALIGNMENT)
        fused = Pose2D(130.0, -7.0, 8.0)
        with patch(
            "components.radar_driver.fuse_wall_observation",
            return_value=WallFusionResult(True, True, None, fused),
        ):
            radar.process_packet(_packet())
        alignment = radar.get_alignment()
        self.assert_pose_equal(alignment.pose_to_global(radar.odometry.pose), fused)


if __name__ == "__main__":
    unittest.main()
