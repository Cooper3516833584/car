import threading
import unittest

from main import (
    MissionPhase,
    ROAD_PATROL_ENFORCE_FIELD_BOUNDS,
    ROAD_PATROL_MAP_MARGIN_CM,
    RETURN_X_CM,
    RETURN_Y_CM,
    TARGET_HOLD_SECONDS,
    TARGET_HEADING_DEG,
    TARGET_X_CM,
    TARGET_Y_CM,
    execute_point_return_mission,
    RoadPatrolCarApplication,
)
from components.fleet_models import AckReason, AckStatus


class RoadPatrolMainTests(unittest.TestCase):
    def make_start_callback_app(self, phase=MissionPhase.READY, running=False):
        app = RoadPatrolCarApplication.__new__(RoadPatrolCarApplication)
        app._lock = threading.RLock()
        app._ready = True
        app._phase = phase
        app._mission_running = running
        app._mission_start_event = threading.Event()
        return app

    def test_default_target_is_easy_to_edit_at_module_top(self):
        self.assertEqual((230.0, 40.0), (TARGET_X_CM, TARGET_Y_CM))

    def test_fixed_target_mission_does_not_enforce_fitted_field_bounds(self):
        self.assertFalse(ROAD_PATROL_ENFORCE_FIELD_BOUNDS)
        self.assertGreaterEqual(ROAD_PATROL_MAP_MARGIN_CM, 250.0)

    def test_route_visits_target_holds_three_seconds_and_returns_without_heading(self):
        calls = []
        waits = []

        def navigate(x_cm, y_cm, heading_deg, phase):
            calls.append((x_cm, y_cm, heading_deg, phase))
            return True

        succeeded = execute_point_return_mission(
            navigate,
            lambda seconds: waits.append(seconds) or False,
        )

        self.assertTrue(succeeded)
        self.assertEqual(
            [
                (TARGET_X_CM, TARGET_Y_CM, TARGET_HEADING_DEG, MissionPhase.OUTBOUND),
                (RETURN_X_CM, RETURN_Y_CM, None, MissionPhase.RETURNING),
            ],
            calls,
        )
        self.assertEqual([TARGET_HOLD_SECONDS], waits)

    def test_failed_outbound_does_not_hold_or_attempt_return(self):
        calls = []
        waits = []

        self.assertFalse(
            execute_point_return_mission(
                lambda x, y, heading, phase: calls.append((x, y, heading, phase)) or False,
                lambda seconds: waits.append(seconds) or False,
            )
        )
        self.assertEqual(1, len(calls))
        self.assertEqual([], waits)

    def test_stop_during_hold_prevents_return(self):
        calls = []

        self.assertFalse(
            execute_point_return_mission(
                lambda x, y, heading, phase: calls.append((x, y, heading, phase)) or True,
                lambda _seconds: True,
            )
        )
        self.assertEqual(1, len(calls))

    def test_start_command_arms_every_ready_mission(self):
        app = self.make_start_callback_app()

        first = app._fleet_start_mission()
        app._mission_start_event.clear()
        app._phase = MissionPhase.COMPLETED
        second = app._fleet_start_mission()

        self.assertEqual(AckStatus.COMPLETED, first.status)
        self.assertEqual(AckStatus.COMPLETED, second.status)
        self.assertTrue(app._mission_start_event.is_set())

    def test_start_command_is_rejected_while_mission_runs(self):
        app = self.make_start_callback_app(
            phase=MissionPhase.OUTBOUND,
            running=True,
        )

        result = app._fleet_start_mission()

        self.assertEqual(AckStatus.REJECTED, result.status)
        self.assertEqual(AckReason.BUSY, result.reason)
        self.assertFalse(app._mission_start_event.is_set())


if __name__ == "__main__":
    unittest.main()
