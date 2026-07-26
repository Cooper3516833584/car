#!/usr/bin/env python3
"""Turn-and-drive coordinate navigation using in-place rotation."""

from __future__ import annotations

import logging
import math
from pathlib import Path
import signal
import sys
import threading
import time

from components.grid_rescue_mission import InPlaceDifferentialTurn
from components.navigation import NavigationGoal, NavigationState
from components.radar_driver import RadarMount
from main import (
    CarMainApplication,
    MainConfig,
    build_argument_parser,
    configure_logging,
    default_log_dir,
    shutdown_logging,
)


LOG = logging.getLogger("car-main")
MAX_CORRECTION_ITERATIONS = 3
CORRECTION_THRESHOLD_CM = 15.0
FINAL_TOLERANCE_CM = 10.0
APPROACH_TOLERANCE_CM = 25.0
STEP_TIMEOUT_S = 90.0


class GridCoordinateApplication(CarMainApplication):
    """Coordinate navigation with explicit turn-to-heading before each drive."""

    def __init__(self, config: MainConfig) -> None:
        if not config.allow_in_place_rotation:
            raise ValueError(
                "grid coordinate main requires allow_in_place_rotation=True"
            )

        super().__init__(config, hmac_key=None)
        self._task_lock = threading.Lock()
        self._task_active = False
        self._task_condition = threading.Condition()
        self._task_terminal_generation = 0
        self._task_terminal_state = None

        self.pivot_turn = InPlaceDifferentialTurn(
            self.navigation.drive,
            pose_provider=lambda: self.navigation.pose,
            on_motion_changed=self.radar.set_motion_hint,
            stop_requested=lambda: self._stop_requested.is_set(),
        )
        LOG.info("grid coordinate navigation initialized with in-place rotation")

    def _on_navigation_state(self, state: NavigationState, reason: str) -> None:
        super()._on_navigation_state(state, reason)
        if state in (
            NavigationState.ARRIVED,
            NavigationState.FAILED,
            NavigationState.BLOCKED,
        ):
            with self._task_condition:
                self._task_terminal_state = state
                self._task_terminal_generation += 1
                self._task_condition.notify_all()

    def _execute_coordinate_task(
        self,
        x_cm: float,
        y_cm: float,
        final_heading_deg: float | None,
    ) -> tuple[bool, str]:
        """Turn toward goal, drive, optionally correct, and return success."""

        with self._task_lock:
            if self._task_active:
                return False, "another task is already active"
            self._task_active = True

        try:
            return self._execute_task_impl(x_cm, y_cm, final_heading_deg)
        finally:
            with self._task_lock:
                self._task_active = False

    def _execute_task_impl(
        self,
        x_cm: float,
        y_cm: float,
        final_heading_deg: float | None,
    ) -> tuple[bool, str]:
        calibration = self.navigation.calibration
        if calibration is None:
            return False, "startup calibration incomplete"

        if not calibration.contains_point(x_cm, y_cm):
            return False, "goal outside fitted field boundary"

        iteration = 0
        while iteration < MAX_CORRECTION_ITERATIONS:
            if self._stop_requested.is_set():
                return False, "task cancelled"

            pose = self.navigation.pose
            if pose is None:
                return False, "no radar pose available"

            dx_cm = x_cm - pose.x_cm
            dy_cm = y_cm - pose.y_cm
            distance_cm = math.hypot(dx_cm, dy_cm)

            # Close enough to goal already?
            if distance_cm <= FINAL_TOLERANCE_CM:
                if final_heading_deg is None:
                    LOG.info(
                        "coordinate task complete without movement distance=%.1fcm",
                        distance_cm,
                    )
                    return True, "already at goal"
                heading_error = abs(
                    self._signed_heading_error(final_heading_deg, pose.heading_deg)
                )
                if heading_error <= 8.0:
                    LOG.info(
                        "coordinate task complete distance=%.1fcm heading_error=%.1fdeg",
                        distance_cm,
                        heading_error,
                    )
                    return True, "already at goal with correct heading"

            # Compute bearing to target
            bearing_rad = math.atan2(dy_cm, dx_cm)
            bearing_deg = math.degrees(bearing_rad)

            # Turn to bearing (always turn for new segments)
            if iteration == 0 or distance_cm > CORRECTION_THRESHOLD_CM:
                LOG.info(
                    "coordinate task iteration=%d turning to bearing=%.1fdeg "
                    "current_pose=(%.1f,%.1f,%.1fdeg) goal=(%.1f,%.1f) distance=%.1fcm",
                    iteration + 1,
                    bearing_deg,
                    pose.x_cm,
                    pose.y_cm,
                    pose.heading_deg,
                    x_cm,
                    y_cm,
                    distance_cm,
                )
                # Diagnostic: check if in-place rotation is enabled
                if not self.navigation.drive.rear_motors.allow_in_place_rotation:
                    LOG.error(
                        "DIAGNOSTIC: rear motors do NOT have allow_in_place_rotation enabled! "
                        "This will cause the turn_to() call to fail."
                    )
                else:
                    LOG.info("DIAGNOSTIC: rear motors have allow_in_place_rotation enabled")

                LOG.info("DIAGNOSTIC: about to call pivot_turn.turn_to(%.1fdeg)", bearing_deg)
                try:
                    self.pivot_turn.turn_to(bearing_deg)
                    LOG.info("DIAGNOSTIC: pivot_turn.turn_to() completed successfully")
                except Exception as exc:
                    LOG.error("in-place turn failed: %s", exc, exc_info=True)
                    return False, f"turn failed: {exc}"

            # Decide whether this segment should enforce final heading
            use_final_heading = (
                final_heading_deg is not None
                and distance_cm <= APPROACH_TOLERANCE_CM
            )
            heading_constraint = final_heading_deg if use_final_heading else None

            # Navigate to goal
            LOG.info(
                "coordinate task iteration=%d navigating to (%.1f,%.1f) "
                "final_heading=%s",
                iteration + 1,
                x_cm,
                y_cm,
                "none" if heading_constraint is None else f"{heading_constraint:.1f}deg",
            )

            success = self._navigate_and_wait(
                x_cm,
                y_cm,
                heading_constraint,
            )

            if not success:
                terminal_state = self._task_terminal_state
                reason_map = {
                    NavigationState.FAILED: "navigation failed",
                    NavigationState.BLOCKED: "path blocked",
                }
                return False, reason_map.get(
                    terminal_state,
                    "navigation did not arrive",
                )

            # Check if we need correction
            pose = self.navigation.pose
            if pose is None:
                return False, "lost radar pose after navigation"

            dx_cm = x_cm - pose.x_cm
            dy_cm = y_cm - pose.y_cm
            residual_cm = math.hypot(dx_cm, dy_cm)

            if residual_cm <= FINAL_TOLERANCE_CM:
                if final_heading_deg is None:
                    LOG.info(
                        "coordinate task complete at iteration=%d residual=%.1fcm",
                        iteration + 1,
                        residual_cm,
                    )
                    return True, "arrived"
                heading_error = abs(
                    self._signed_heading_error(final_heading_deg, pose.heading_deg)
                )
                if heading_error <= 8.0:
                    LOG.info(
                        "coordinate task complete at iteration=%d residual=%.1fcm "
                        "heading_error=%.1fdeg",
                        iteration + 1,
                        residual_cm,
                        heading_error,
                    )
                    return True, "arrived with correct heading"

            if residual_cm <= CORRECTION_THRESHOLD_CM:
                LOG.info(
                    "coordinate task complete within correction threshold "
                    "iteration=%d residual=%.1fcm",
                    iteration + 1,
                    residual_cm,
                )
                return True, "arrived within correction threshold"

            # Need another correction iteration
            iteration += 1
            LOG.info(
                "coordinate task requires correction iteration=%d residual=%.1fcm "
                "current=(%.1f,%.1f,%.1fdeg)",
                iteration + 1,
                residual_cm,
                pose.x_cm,
                pose.y_cm,
                pose.heading_deg,
            )

        return False, f"max correction iterations ({MAX_CORRECTION_ITERATIONS}) exceeded"

    def _navigate_and_wait(
        self,
        x_cm: float,
        y_cm: float,
        final_heading_deg: float | None,
    ) -> bool:
        """Submit navigation goal and wait for terminal state."""

        with self._task_condition:
            generation = self._task_terminal_generation

        goal = NavigationGoal(x_cm, y_cm, final_heading_deg)
        try:
            self._submit_console_goal(goal)
        except Exception as exc:
            LOG.error("coordinate goal rejected: %s", exc)
            return False

        deadline = time.monotonic() + STEP_TIMEOUT_S
        with self._task_condition:
            while self._task_terminal_generation == generation:
                if self._stop_requested.is_set():
                    self._cancel_from_console()
                    return False

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._cancel_from_console()
                    LOG.error("coordinate navigation step timeout")
                    return False

                self._task_condition.wait(min(0.2, remaining))

            return self._task_terminal_state is NavigationState.ARRIVED

    @staticmethod
    def _signed_heading_error(target_deg: float, current_deg: float) -> float:
        """Return signed error in [-180, 180] with CCW positive."""
        error = (target_deg - current_deg) % 360.0
        if error > 180.0:
            error -= 360.0
        return error

    def _console_main(self) -> None:
        """Override console to add turn-and-drive coordinate input."""

        print("\nGrid Coordinate Navigation Ready")
        print("Field bounds: x=[{:.1f},{:.1f}] y=[{:.1f},{:.1f}] cm".format(
            self.navigation.calibration.min_x_cm,
            self.navigation.calibration.max_x_cm,
            self.navigation.calibration.min_y_cm,
            self.navigation.calibration.max_y_cm,
        ))
        print("\nCommands:")
        print("  <x_cm> <y_cm> [heading_deg]  - Navigate with turn-and-drive")
        print("  status                        - Show current state")
        print("  stop                          - Cancel active navigation")
        print("  help                          - Show this message")
        print("  quit                          - Exit application")
        print()

        while not self._stop_requested.is_set():
            try:
                line = input("> ").strip()
            except EOFError:
                break

            if not line:
                continue

            parts = line.split()
            command = parts[0].lower()

            if command == "quit":
                self.request_stop()
                break
            elif command == "help":
                print("Commands: <x> <y> [heading] | status | stop | quit")
            elif command == "status":
                self._print_status()
            elif command == "stop":
                if self.navigation.active:
                    self._cancel_from_console()
                    print("Navigation cancelled")
                else:
                    print("No active navigation")
            else:
                # Try to parse as coordinate command
                try:
                    if len(parts) < 2:
                        print("Error: need at least x_cm and y_cm")
                        continue

                    x_cm = float(parts[0])
                    y_cm = float(parts[1])
                    final_heading_deg = None if len(parts) < 3 else float(parts[2])

                    if final_heading_deg is not None:
                        if not 0 <= final_heading_deg < 360:
                            print("Error: heading must be in [0, 360)")
                            continue

                    print("Executing turn-and-drive to ({:.1f}, {:.1f}) heading={}".format(
                        x_cm,
                        y_cm,
                        "none" if final_heading_deg is None else f"{final_heading_deg:.1f}deg",
                    ))

                    success, reason = self._execute_coordinate_task(
                        x_cm,
                        y_cm,
                        final_heading_deg,
                    )

                    if success:
                        print(f"Task completed: {reason}")
                    else:
                        print(f"Task failed: {reason}")

                except ValueError as exc:
                    print(f"Error: invalid coordinate format: {exc}")
                except Exception as exc:
                    LOG.exception("coordinate task error")
                    print(f"Error: {exc}")

    def _print_status(self) -> None:
        """Print current navigation state and pose."""

        state = self.navigation.state
        pose = self.navigation.pose

        print(f"State: {state.name}")
        if pose is not None:
            age_s = time.monotonic() - pose.timestamp_s
            print(f"Pose: ({pose.x_cm:.1f}, {pose.y_cm:.1f}, {pose.heading_deg:.1f}deg) "
                  f"age={age_s:.2f}s")
        else:
            print("Pose: unavailable")

        with self._task_lock:
            print(f"Task active: {self._task_active}")


def main(argv=None) -> int:
    parser = build_argument_parser()
    parser.description = __doc__
    args = parser.parse_args(argv)

    requested_log_dir = default_log_dir() if args.log_dir is None else Path(args.log_dir)
    try:
        configure_logging(requested_log_dir, args.log_level)
    except OSError as exc:
        print(
            f"cannot create detailed log in {requested_log_dir}: {exc}",
            file=sys.stderr,
        )
        return 2

    app = None
    try:
        config = MainConfig(
            radar_port=args.radar_port,
            link_port=args.link_port,
            radar_mount=RadarMount(
                args.radar_x_cm,
                args.radar_y_cm,
                args.radar_yaw_cw_deg,
            ),
            startup_scan_count=args.startup_scans,
            calibration_timeout_s=args.calibration_timeout,
            allow_reverse=args.allow_reverse,
            allow_in_place_rotation=True,
            console_enabled=not args.no_console,
        )
        app = GridCoordinateApplication(config)

        def stop_handler(signum, frame) -> None:
            LOG.info("received signal %s; stopping grid coordinate main", signum)
            app.request_stop()

        signal.signal(signal.SIGINT, stop_handler)
        signal.signal(signal.SIGTERM, stop_handler)
        app.run()
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception:
        LOG.exception("grid coordinate main failed")
        return 1
    finally:
        try:
            if app is not None:
                app.close()
        finally:
            shutdown_logging()


if __name__ == "__main__":
    raise SystemExit(main())
