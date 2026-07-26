"""Unit tests for grid_coordinate_main turn-and-drive navigation."""

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
        self.calibration = RectangleFieldCalibration(
            min_x_cm=-100.0,
            max_x_cm=300.0,
            min_y_cm=-100.0,
            max_y_cm=200.0,
            wall_reference=None,
            local_to_global=None,
        )
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


class TestGridCoordinateMain(unittest.TestCase):
    def setUp(self):
        self.config = MainConfig(
            allow_in_place_rotation=True,
            console_enabled=False,
        )

    def _setup_mocked_app(self, app_self, config, hmac_key=None):
        """Helper to initialize mocked CarMainApplication for tests."""
        app_self._stop_requested = threading.Event()
        app_self._lock = threading.RLock()
        app_self._ready = False
        app_self.config = config
        app_self.coordinate_navigation = MagicMock()
        app_self.radar = MagicMock()
        # Create a property-like access for navigation
        fake_nav = FakeNavigation()
        app_self.coordinate_navigation.navigation = fake_nav
        app_self.coordinate_navigation.radar = app_self.radar

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    def test_requires_in_place_rotation_enabled(self, mock_super_init):
        """Initialization must reject config without in-place rotation."""
        mock_super_init.side_effect = self._setup_mocked_app

        with self.assertRaises(ValueError) as ctx:
            config = MainConfig(allow_in_place_rotation=False, console_enabled=False)
            GridCoordinateApplication(config)

        self.assertIn("allow_in_place_rotation", str(ctx.exception))

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    def test_turn_and_drive_sequence(self, mock_super_init):
        """Task must turn to bearing before each drive segment."""
        mock_super_init.side_effect = self._setup_mocked_app

        app = GridCoordinateApplication(self.config)
        fake_nav = app.navigation
        fake_nav.pose = NavigationPose(0.0, 0.0, 0.0)
        fake_nav.on_state_changed = app._on_navigation_state

        fake_turn = FakePivotTurn()
        app.pivot_turn = fake_turn

        # Simulate navigation arriving at exact goal in one iteration
        def simulate_success(*args, **kwargs):
            fake_nav.simulate_arrival(NavigationPose(100.0, 50.0, 45.0))

        app._submit_console_goal = simulate_success

        success, reason = app._execute_coordinate_task(100.0, 50.0, None)

        self.assertTrue(success)
        self.assertEqual(len(fake_turn.turn_calls), 1)
        # Bearing from (0, 0) to (100, 50) is atan2(50, 100) ≈ 26.57 deg
        expected_bearing = math.degrees(math.atan2(50.0, 100.0))
        self.assertAlmostEqual(fake_turn.turn_calls[0], expected_bearing, places=1)
        self.assertEqual(len(fake_nav.goals), 1)
        self.assertAlmostEqual(fake_nav.goals[0].x_cm, 100.0)
        self.assertAlmostEqual(fake_nav.goals[0].y_cm, 50.0)
        self.assertIsNone(fake_nav.goals[0].final_heading_deg)

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    def test_final_heading_only_on_last_segment(self, mock_super_init):
        """Final heading constraint must only apply when close to goal."""
        mock_super_init.side_effect = self._setup_mocked_app

        app = GridCoordinateApplication(self.config)
        fake_nav = app.navigation
        fake_nav.pose = NavigationPose(0.0, 0.0, 0.0)
        fake_nav.on_state_changed = app._on_navigation_state

        fake_turn = FakePivotTurn()
        app.pivot_turn = fake_turn

        call_count = 0

        def simulate_approach(*args, **kwargs):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                # First segment: arrive far from goal (> APPROACH_TOLERANCE_CM)
                fake_nav.simulate_arrival(NavigationPose(50.0, 0.0, 0.0))
            else:
                # Second segment: arrive at goal
                fake_nav.simulate_arrival(NavigationPose(100.0, 0.0, 90.0))

        app._submit_console_goal = simulate_approach

        success, reason = app._execute_coordinate_task(100.0, 0.0, 90.0)

        self.assertTrue(success)
        self.assertEqual(len(fake_nav.goals), 2)
        # First segment should not have final heading (distance > APPROACH_TOLERANCE_CM)
        self.assertIsNone(fake_nav.goals[0].final_heading_deg)
        # Second segment should have final heading (distance < APPROACH_TOLERANCE_CM)
        self.assertAlmostEqual(fake_nav.goals[1].final_heading_deg, 90.0)

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    def test_correction_iteration_on_residual(self, mock_super_init):
        """Task must retry if residual exceeds correction threshold."""
        mock_super_init.side_effect = self._setup_mocked_app

        app = GridCoordinateApplication(self.config)
        fake_nav = app.navigation
        fake_nav.pose = NavigationPose(0.0, 0.0, 0.0)
        fake_nav.on_state_changed = app._on_navigation_state

        fake_turn = FakePivotTurn()
        app.pivot_turn = fake_turn

        call_count = 0

        def simulate_with_residual(*args, **kwargs):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                # First attempt: arrive with 20cm residual
                fake_nav.simulate_arrival(NavigationPose(80.0, 0.0, 0.0))
            else:
                # Correction: arrive at goal
                fake_nav.simulate_arrival(NavigationPose(100.0, 0.0, 0.0))

        app._submit_console_goal = simulate_with_residual

        success, reason = app._execute_coordinate_task(100.0, 0.0, None)

        self.assertTrue(success)
        self.assertEqual(len(fake_turn.turn_calls), 2)
        self.assertEqual(len(fake_nav.goals), 2)

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    def test_rejects_concurrent_tasks(self, mock_super_init):
        """Only one task may be active at a time."""
        mock_super_init.side_effect = self._setup_mocked_app

        app = GridCoordinateApplication(self.config)
        fake_nav = app.navigation
        fake_nav.pose = NavigationPose(0.0, 0.0, 0.0)
        fake_nav.on_state_changed = app._on_navigation_state

        fake_turn = FakePivotTurn()
        app.pivot_turn = fake_turn

        # Make first task block
        first_started = threading.Event()
        first_continue = threading.Event()

        def blocking_submit(*args, **kwargs):
            first_started.set()
            first_continue.wait()
            fake_nav.simulate_arrival(NavigationPose(10.0, 0.0, 0.0))

        app._submit_console_goal = blocking_submit

        # Start first task in thread
        result1 = []

        def task1():
            result1.append(app._execute_coordinate_task(10.0, 0.0, None))

        thread1 = threading.Thread(target=task1, daemon=True)
        thread1.start()
        first_started.wait(timeout=1.0)

        # Try to start second task while first is active
        success2, reason2 = app._execute_coordinate_task(20.0, 0.0, None)

        self.assertFalse(success2)
        self.assertIn("already active", reason2)

        # Let first task complete
        first_continue.set()
        thread1.join(timeout=1.0)
        self.assertEqual(len(result1), 1)
        self.assertTrue(result1[0][0])

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    def test_rejects_goal_outside_field(self, mock_super_init):
        """Task must reject goals outside calibrated field bounds."""
        mock_super_init.side_effect = self._setup_mocked_app

        app = GridCoordinateApplication(self.config)
        fake_nav = app.navigation
        fake_nav.pose = NavigationPose(0.0, 0.0, 0.0)

        fake_turn = FakePivotTurn()
        app.pivot_turn = fake_turn

        # Goal outside x bounds
        success, reason = app._execute_coordinate_task(500.0, 0.0, None)
        self.assertFalse(success)
        self.assertIn("outside", reason.lower())

        # Goal outside y bounds
        success, reason = app._execute_coordinate_task(0.0, 500.0, None)
        self.assertFalse(success)
        self.assertIn("outside", reason.lower())

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    def test_handles_turn_failure(self, mock_super_init):
        """Task must fail gracefully if in-place turn raises exception."""
        mock_super_init.side_effect = self._setup_mocked_app

        app = GridCoordinateApplication(self.config)
        fake_nav = app.navigation
        fake_nav.pose = NavigationPose(0.0, 0.0, 0.0)

        fake_turn = FakePivotTurn()
        fake_turn.should_raise = RuntimeError("turn timeout")
        app.pivot_turn = fake_turn

        success, reason = app._execute_coordinate_task(100.0, 0.0, None)

        self.assertFalse(success)
        self.assertIn("turn failed", reason.lower())

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    def test_handles_navigation_failure(self, mock_super_init):
        """Task must detect and report navigation FAILED state."""
        mock_super_init.side_effect = self._setup_mocked_app

        app = GridCoordinateApplication(self.config)
        fake_nav = app.navigation
        fake_nav.pose = NavigationPose(0.0, 0.0, 0.0)
        fake_nav.on_state_changed = app._on_navigation_state

        fake_turn = FakePivotTurn()
        app.pivot_turn = fake_turn

        def simulate_fail(*args, **kwargs):
            fake_nav.simulate_failure()

        app._submit_console_goal = simulate_fail

        success, reason = app._execute_coordinate_task(100.0, 0.0, None)

        self.assertFalse(success)
        self.assertIn("failed", reason.lower())

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    def test_handles_navigation_blocked(self, mock_super_init):
        """Task must detect and report navigation BLOCKED state."""
        mock_super_init.side_effect = self._setup_mocked_app

        app = GridCoordinateApplication(self.config)
        fake_nav = app.navigation
        fake_nav.pose = NavigationPose(0.0, 0.0, 0.0)
        fake_nav.on_state_changed = app._on_navigation_state

        fake_turn = FakePivotTurn()
        app.pivot_turn = fake_turn

        def simulate_block(*args, **kwargs):
            fake_nav.simulate_blocked()

        app._submit_console_goal = simulate_block

        success, reason = app._execute_coordinate_task(100.0, 0.0, None)

        self.assertFalse(success)
        self.assertIn("blocked", reason.lower())

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    def test_max_correction_iterations(self, mock_super_init):
        """Task must fail after maximum correction iterations."""
        mock_super_init.side_effect = self._setup_mocked_app

        app = GridCoordinateApplication(self.config)
        fake_nav = app.navigation
        fake_nav.pose = NavigationPose(0.0, 0.0, 0.0)
        fake_nav.on_state_changed = app._on_navigation_state

        fake_turn = FakePivotTurn()
        app.pivot_turn = fake_turn

        # Always arrive with large residual
        def simulate_persistent_residual(*args, **kwargs):
            fake_nav.pose = NavigationPose(
                fake_nav.pose.x_cm + 20.0,
                fake_nav.pose.y_cm,
                0.0,
            )
            fake_nav.simulate_arrival(fake_nav.pose)

        app._submit_console_goal = simulate_persistent_residual

        success, reason = app._execute_coordinate_task(200.0, 0.0, None)

        self.assertFalse(success)
        self.assertIn("max correction iterations", reason.lower())

    @patch("grid_coordinate_main.CarMainApplication.__init__")
    def test_already_at_goal_skips_movement(self, mock_super_init):
        """Task must succeed immediately if already at goal."""
        mock_super_init.side_effect = self._setup_mocked_app

        app = GridCoordinateApplication(self.config)
        fake_nav = app.navigation
        fake_nav.pose = NavigationPose(100.0, 50.0, 45.0)

        fake_turn = FakePivotTurn()
        app.pivot_turn = fake_turn

        success, reason = app._execute_coordinate_task(100.0, 50.0, None)

        self.assertTrue(success)
        self.assertIn("already at goal", reason.lower())
        self.assertEqual(len(fake_turn.turn_calls), 0)


if __name__ == "__main__":
    unittest.main()
