"""Unit tests for the trusted radar/navigation map component."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components import (  # noqa: E402
    DroneGlobalAlignment,
    ICPResult,
    Pose2D,
    RadarLocalizationUpdate,
    RadarScan,
    RectangularWallReference,
    RectangleFieldCalibration,
    TrustedNavigationMap,
    TrustedNavigationMapConfig,
    VehicleGeometry,
    WallFusionResult,
    WallFusionStatus,
)
from components.radar_driver import RadarOdometryUpdate  # noqa: E402


def make_calibration() -> RectangleFieldCalibration:
    identity = DroneGlobalAlignment(0.0, 0.0, 0.0)
    return RectangleFieldCalibration(
        identity,
        RectangularWallReference(identity, -100.0, -50.0, 200.0, 150.0),
        Pose2D(),
        -100.0,
        200.0,
        -50.0,
        150.0,
        0.0,
        4,
    )


def make_update(
    pose: Pose2D,
    *,
    error_cm: float = 1.0,
    points: tuple[tuple[float, float], ...] = (),
    wall_fusion: WallFusionResult | None = None,
) -> RadarLocalizationUpdate:
    return RadarLocalizationUpdate(
        RadarScan((), 1000, 3600),
        RadarOdometryUpdate(
            pose,
            True,
            True,
            ICPResult(Pose2D(), 100, error_cm, 3),
        ),
        pose,
        points,
        wall_fusion,
    )


class TrustedNavigationMapTests(unittest.TestCase):
    def make_component(self) -> TrustedNavigationMap:
        component = TrustedNavigationMap(
            TrustedNavigationMapConfig(update_interval_s=0.5),
            VehicleGeometry(),
        )
        component.initialize(make_calibration(), [], pose=Pose2D(), now=10.0)
        return component

    def test_rejects_pose_jump_icp_error_and_field_exit(self) -> None:
        component = self.make_component()

        jump = component.rejection_reason(make_update(Pose2D(30.0, 0.0, 0.0)))
        icp = component.rejection_reason(
            make_update(Pose2D(2.0, 0.0, 0.0), error_cm=10.1)
        )
        outside = component.rejection_reason(
            make_update(Pose2D(195.0, 0.0, 0.0))
        )

        self.assertIn("translation jump", jump or "")
        self.assertIn("ICP error", icp or "")
        self.assertIn("footprint outside", outside or "")

    def test_initial_grid_forbids_everything_outside_fitted_polygon(self) -> None:
        component = self.make_component()
        grid = component.grid
        assert grid is not None

        self.assertTrue(grid.is_occupied(*grid.world_to_cell(-110.0, 0.0)))
        self.assertFalse(grid.is_occupied(*grid.world_to_cell(0.0, 0.0)))

    def test_ingest_filters_self_returns_and_refreshes_only_when_due(self) -> None:
        component = self.make_component()
        result = component.ingest(
            make_update(
                Pose2D(2.0, 0.0, 0.0),
                points=((2.0, 0.0), (60.0, 20.0)),
            ),
            now=10.1,
        )

        self.assertIsNone(result.refreshed_grid)
        self.assertEqual(result.retained_points, 1)
        refreshed = component.refresh_grid(now=10.5)
        self.assertIsNotNone(refreshed)
        self.assertEqual(component.last_pose, Pose2D(2.0, 0.0, 0.0))

    def test_hard_rejected_wall_scan_does_not_enter_trusted_map(self) -> None:
        component = self.make_component()
        pose = Pose2D(2.0, 0.0, 0.0)
        wall = WallFusionResult(
            True,
            False,
            None,
            pose,
            "wall residual gate",
            status=WallFusionStatus.HARD_REJECTED,
        )

        result = component.ingest(
            make_update(pose, points=((60.0, 20.0),), wall_fusion=wall),
            now=10.1,
        )

        self.assertTrue(result.wall_hard_rejected)
        self.assertEqual(component.point_map.cells(), [])


if __name__ == "__main__":
    unittest.main()
