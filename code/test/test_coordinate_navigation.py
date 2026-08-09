"""Hardware-free tests for the coordinate-level navigation component."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components import (  # noqa: E402
    CoordinateGoalRejected,
    CoordinateGoalRejectReason,
    CoordinateNavigation,
    CoordinateNavigationConfig,
    DroneGlobalAlignment,
    NavigationGoal,
    Pose2D,
    RadarMount,
    RectangleFieldCalibration,
    RectangularWallReference,
)


def make_calibration() -> RectangleFieldCalibration:
    identity = DroneGlobalAlignment(0.0, 0.0, 0.0)
    return RectangleFieldCalibration(
        identity,
        RectangularWallReference(identity, -100.0, -80.0, 200.0, 160.0),
        Pose2D(),
        -100.0,
        200.0,
        -80.0,
        160.0,
        0.0,
        4,
    )


class FakeNavigation:
    def __init__(self) -> None:
        self.active = False
        self.grid = None
        self.calls: list[tuple[float, float, float | None, float, float]] = []

    def set_map(self, grid) -> bool:
        changed = grid != self.grid
        self.grid = grid
        return changed

    def navigate_to(
        self,
        x_cm,
        y_cm,
        final_heading_deg=None,
        *,
        position_tolerance_cm=10.0,
        heading_tolerance_deg=8.0,
    ):
        self.calls.append(
            (
                x_cm,
                y_cm,
                final_heading_deg,
                position_tolerance_cm,
                heading_tolerance_deg,
            )
        )
        self.active = True


class CoordinateNavigationTests(unittest.TestCase):
    def make_component(
        self,
        *,
        enforce_field_bounds: bool = True,
        map_margin_cm: float = 15.0,
    ) -> tuple[CoordinateNavigation, FakeNavigation]:
        component = CoordinateNavigation(
            CoordinateNavigationConfig(
                radar_mount=RadarMount(0.01, 0.0, 0.0),
                enforce_field_bounds=enforce_field_bounds,
                map_margin_cm=map_margin_cm,
            )
        )
        calibration = make_calibration()
        component.trusted_map.initialize(calibration, [], pose=Pose2D())
        fake = FakeNavigation()
        component.navigation = fake  # type: ignore[assignment]
        component._ready = True
        return component, fake

    def test_single_call_forwards_start_relative_pose(self) -> None:
        component, navigation = self.make_component()

        goal = component.navigate_to(80.0, 40.0, 90.0)

        self.assertEqual(goal, NavigationGoal(80.0, 40.0, 90.0))
        self.assertEqual(navigation.calls, [(80.0, 40.0, 90.0, 5.0, 8.0)])

    def test_goal_object_tolerances_are_preserved(self) -> None:
        component, navigation = self.make_component()
        goal = NavigationGoal(80.0, 40.0, 90.0, 5.0, 3.0)

        component.navigate(goal)

        self.assertEqual(navigation.calls[-1], (80.0, 40.0, 90.0, 5.0, 3.0))

    def test_outside_goal_is_rejected_before_control(self) -> None:
        component, navigation = self.make_component()

        with self.assertRaises(CoordinateGoalRejected) as caught:
            component.navigate_to(250.0, 0.0, 0.0)

        self.assertIs(caught.exception.reason, CoordinateGoalRejectReason.OUTSIDE_FIELD)
        self.assertEqual(navigation.calls, [])

    def test_outside_goal_is_forwarded_when_field_bounds_are_disabled(self) -> None:
        component, navigation = self.make_component(
            enforce_field_bounds=False,
            map_margin_cm=100.0,
        )

        goal = component.navigate_to(250.0, 0.0, 0.0)

        self.assertEqual(goal, NavigationGoal(250.0, 0.0, 0.0))
        self.assertEqual(navigation.calls, [(250.0, 0.0, 0.0, 5.0, 8.0)])
if __name__ == "__main__":
    unittest.main()
