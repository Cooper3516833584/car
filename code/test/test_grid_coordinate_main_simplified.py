"""Simplified unit tests for grid_coordinate_main turn-and-drive navigation."""

from __future__ import annotations

import math
import threading
import time
import unittest
from unittest.mock import MagicMock, Mock, patch

from components.navigation import NavigationGoal, NavigationPose, NavigationState
from components.radar_driver import RectangleFieldCalibration
from grid_coordinate_main import GridCoordinateApplication, MainConfig


class FakePivotTurn:
    """Mock in-place turn that records calls."""

    def __init__(self):
        self.turn_calls = []
        self.should_raise = None

    def turn_to(self, heading_deg: float) -> None:
        self.turn_calls.append(heading_deg)
        if self.should_raise is not None:
            raise self.should_raise


class FakeNavigation:
    """Mock navigation that can simulate terminal states."""

    def __init__(self):
        self.pose = NavigationPose(0.0, 0.0, 0.0)
        self.state = NavigationState.IDLE
        self.active = False
        # Mock calibration instead of creating real object
        self.calibration = MagicMock()
        self.calibration.contains_point = lambda x, y: -100 <= x <= 300 and -100 <= y <= 200
        self.calibration.min_x_cm = -100.0
        self.calibration.max_x_cm = 300.0
        self.calibration.min_y_cm = -100.0
        self.calibration.max_y_cm = 200.0
        self.goals = []
        self.on_state_changed = None
        self.drive = MagicMock()

    def set_goal(self, goal: NavigationGoal) -> None:
        self.goals.append(goal)

    def start_navigation(self) -> None:
        self.active = True

    def cancel(self, *, reason: str = "") -> None:
        self.active = False
        self._emit_state(NavigationState.IDLE, reason)

    def simulate_arrival(self, new_pose: NavigationPose) -> None:
        """Simulate navigation completing at a new pose."""
        self.pose = new_pose
        self.active = False
        self._emit_state(NavigationState.ARRIVED, "test arrival")

    def simulate_failure(self) -> None:
        """Simulate navigation failure."""
        self.active = False
        self._emit_state(NavigationState.FAILED, "test failure")

    def simulate_blocked(self) -> None:
        """Simulate navigation blocked."""
        self.active = False
        self._emit_state(NavigationState.BLOCKED, "test blocked")

    def _emit_state(self, state: NavigationState, reason: str) -> None:
        self.state = state
        if self.on_state_changed is not None:
            self.on_state_changed(state, reason)


class TestGridCoordinateMainSimplified(unittest.TestCase):
    """Test grid coordinate main by mocking CoordinateNavigation instead of __init__."""

    def setUp(self):
        self.config = MainConfig(
            allow_in_place_rotation=True,
            console_enabled=False,
        )

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    @patch("grid_coordinate_main.InPlaceDifferentialTurn")
    def test_basic_functionality(self, mock_turn_class, mock_super_init):
        """Verify basic turn-and-drive works with mocked components."""
        # Setup CarMainApplication mock
        def setup_app(config, hmac_key=None):
            import inspect
            frame = inspect.currentframe()
            caller_locals = frame.f_back.f_back.f_locals
            app_self = caller_locals.get('self')

            app_self._stop_requested = threading.Event()
            app_self._lock = threading.RLock()
            app_self._ready = False
            app_self.config = config
            app_self.coordinate_navigation = MagicMock()
            app_self.radar = MagicMock()
            fake_nav = FakeNavigation()
            app_self.coordinate_navigation.navigation = fake_nav

        mock_super_init.side_effect = setup_app

        # Setup turn mock
        fake_turn = FakePivotTurn()
        mock_turn_class.return_value = fake_turn

        # Create app
        app = GridCoordinateApplication(self.config)
        fake_nav = app.navigation
        fake_nav.on_state_changed = app._on_navigation_state

        # Simulate success
        def simulate_success(*args, **kwargs):
            fake_nav.simulate_arrival(NavigationPose(100.0, 50.0, 0.0))

        app._submit_console_goal = simulate_success

        # Execute
        success, reason = app._execute_coordinate_task(100.0, 50.0, None)

        # Verify
        self.assertTrue(success)
        self.assertEqual(len(fake_turn.turn_calls), 1)
        expected_bearing = math.degrees(math.atan2(50.0, 100.0))
        self.assertAlmostEqual(fake_turn.turn_calls[0], expected_bearing, places=1)


if __name__ == "__main__":
    unittest.main()
