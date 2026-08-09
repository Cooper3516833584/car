import unittest

from main import (
    MissionPhase,
    ROAD_PATROL_ENFORCE_FIELD_BOUNDS,
    ROAD_PATROL_MAP_MARGIN_CM,
    RETURN_X_CM,
    RETURN_Y_CM,
    TARGET_HOLD_SECONDS,
    TARGET_X_CM,
    TARGET_Y_CM,
    execute_point_return_mission,
)


class RoadPatrolMainTests(unittest.TestCase):
    def test_default_target_is_easy_to_edit_at_module_top(self):
        self.assertEqual((160.0, 100.0), (TARGET_X_CM, TARGET_Y_CM))

    def test_fixed_target_mission_does_not_enforce_fitted_field_bounds(self):
        self.assertFalse(ROAD_PATROL_ENFORCE_FIELD_BOUNDS)
        self.assertGreaterEqual(ROAD_PATROL_MAP_MARGIN_CM, 250.0)

    def test_route_visits_target_holds_three_seconds_and_returns_without_heading(self):
        calls = []
        waits = []

        def navigate(x_cm, y_cm, phase):
            calls.append((x_cm, y_cm, phase))
            return True

        succeeded = execute_point_return_mission(
            navigate,
            lambda seconds: waits.append(seconds) or False,
        )

        self.assertTrue(succeeded)
        self.assertEqual(
            [
                (TARGET_X_CM, TARGET_Y_CM, MissionPhase.OUTBOUND),
                (RETURN_X_CM, RETURN_Y_CM, MissionPhase.RETURNING),
            ],
            calls,
        )
        self.assertEqual([TARGET_HOLD_SECONDS], waits)

    def test_failed_outbound_does_not_hold_or_attempt_return(self):
        calls = []
        waits = []

        self.assertFalse(
            execute_point_return_mission(
                lambda x, y, phase: calls.append((x, y, phase)) or False,
                lambda seconds: waits.append(seconds) or False,
            )
        )
        self.assertEqual(1, len(calls))
        self.assertEqual([], waits)

    def test_stop_during_hold_prevents_return(self):
        calls = []

        self.assertFalse(
            execute_point_return_mission(
                lambda x, y, phase: calls.append((x, y, phase)) or True,
                lambda _seconds: True,
            )
        )
        self.assertEqual(1, len(calls))


if __name__ == "__main__":
    unittest.main()
