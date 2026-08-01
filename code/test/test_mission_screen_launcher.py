from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mission_screen_launcher import MissionScreenLauncher  # noqa: E402
from radar_center_config import load_radar_center_behind_a_cm  # noqa: E402


class MissionScreenLauncherTests(unittest.TestCase):
    def test_idle_reporter_restarts_around_child_task(self) -> None:
        events = []

        class Reporter:
            def __init__(self, distance_provider):
                self.distance_provider = distance_provider

            def start(self):
                events.append(("start", self.distance_provider()))

            def close(self):
                events.append(("close", self.distance_provider()))

        launcher = MissionScreenLauncher(
            Path("/tasks"), 10.0, idle_reporter_factory=Reporter
        )
        launcher.start_idle_reporting()
        launcher.start_idle_reporting()
        self.assertEqual([("start", 20.0)], events)

        with patch("mission_screen_launcher.Path.is_file", return_value=True), patch(
            "mission_screen_launcher.subprocess.Popen"
        ) as popen:
            process = popen.return_value
            process.poll.return_value = 0
            process.pid = 42
            launcher.schedule(Path("/tasks/main_task1.py"), 1.0, delay_s=0.0)
            launcher.poll(1.0)
            self.assertEqual([("start", 20.0), ("close", 20.0)], events)
            launcher.poll(2.0)

        self.assertEqual(
            [("start", 20.0), ("close", 20.0), ("start", 20.0)],
            events,
        )

    @patch("mission_screen_launcher.time.sleep")
    @patch("mission_screen_launcher.SoundLightAlarm")
    def test_fragmented_case_insensitive_token_schedules_task_one(
        self, alarm_class, sleep
    ) -> None:
        launcher = MissionScreenLauncher(Path("/tasks"), 10.0)
        launcher.receive(b"noise-mis", 1.0)
        launcher.receive(b"sion1\xff", 2.0)
        self.assertIsNotNone(launcher.pending)
        assert launcher.pending is not None
        self.assertEqual(launcher.pending.task_path, Path("/tasks/main_task1.py"))
        self.assertEqual(launcher.pending.due_at, 2.0)
        self.assertFalse(launcher.pending.prelaunch_alarm)
        self.assertEqual(3, alarm_class.return_value.on.call_count)
        self.assertGreaterEqual(alarm_class.return_value.off.call_count, 3)
        self.assertEqual(5, sleep.call_count)

    @patch("mission_screen_launcher.time.sleep")
    @patch("mission_screen_launcher.SoundLightAlarm")
    def test_fragmented_case_insensitive_task_two_is_immediate(
        self, alarm_class, sleep
    ) -> None:
        launcher = MissionScreenLauncher(Path("/tasks"), 10.0)
        launcher.receive(b"noise-mIs", 1.0)
        launcher.receive(b"SiOn2\xff", 2.0)

        self.assertIsNotNone(launcher.pending)
        assert launcher.pending is not None
        self.assertEqual(launcher.pending.task_path, Path("/tasks/main_task2.py"))
        self.assertEqual(launcher.pending.due_at, 2.0)
        self.assertFalse(launcher.pending.prelaunch_alarm)
        self.assertEqual(3, alarm_class.return_value.on.call_count)
        self.assertGreaterEqual(alarm_class.return_value.off.call_count, 3)
        self.assertEqual(5, sleep.call_count)

    @patch("mission_screen_launcher.time.sleep")
    @patch("mission_screen_launcher.SoundLightAlarm")
    def test_later_button_replaces_pending_choice(
        self, _alarm_class, _sleep
    ) -> None:
        launcher = MissionScreenLauncher(Path("/tasks"), 10.0)
        launcher.receive(b"MISSION1", 1.0)
        launcher.receive(b"MISSION2", 3.0)
        assert launcher.pending is not None
        self.assertEqual(launcher.pending.task_path.name, "main_task2.py")
        self.assertEqual(launcher.pending.due_at, 3.0)
        self.assertFalse(launcher.pending.prelaunch_alarm)

    @patch("mission_screen_launcher.Path.is_file", return_value=True)
    @patch("mission_screen_launcher.subprocess.Popen")
    @patch("mission_screen_launcher.time.sleep")
    @patch("mission_screen_launcher.SoundLightAlarm")
    def test_task_two_launches_on_next_poll_without_prelaunch_alarm(
        self, alarm_class, _sleep, popen, _is_file
    ) -> None:
        process = popen.return_value
        process.poll.return_value = None
        process.pid = 42
        launcher = MissionScreenLauncher(Path("/tasks"), 10.0)
        launcher.receive(b"MISSION2", 1.0)

        pending_alarm = launcher.pending.alarm if launcher.pending else None
        self.assertIsNone(pending_alarm)
        launcher.poll(1.0)

        self.assertEqual(3, alarm_class.return_value.on.call_count)
        popen.assert_called_once_with(
            [
                sys.executable,
                str(Path("/tasks/main_task2.py")),
                "--radar-center-behind-a-cm",
                "20",
            ],
            cwd=Path("/tasks"), start_new_session=True,
        )

    def test_fragmented_radar_distance_is_saved_and_passed_to_task(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            task_directory = Path(directory)
            config_path = task_directory / "radar-center.json"
            launcher = MissionScreenLauncher(
                task_directory, 10.0, config_path=config_path
            )

            launcher.receive(b"36", 1.0)
            launcher.receive(b".5\xff\xff\xff", 2.0)

            self.assertEqual(36.5, launcher.radar_center_behind_a_cm)
            self.assertEqual(36.5, load_radar_center_behind_a_cm(config_path))
            self.assertIsNone(launcher.pending)

    def test_saved_radar_distance_survives_launcher_restart(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            task_directory = Path(directory)
            config_path = task_directory / "radar-center.json"
            first = MissionScreenLauncher(
                task_directory, 10.0, config_path=config_path
            )
            first.receive(b"36.5", 1.0)

            second = MissionScreenLauncher(
                task_directory, 10.0, config_path=config_path
            )

            self.assertEqual(36.5, second.radar_center_behind_a_cm)

    def test_distance_change_is_ignored_while_task_is_running(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            task_directory = Path(directory)
            config_path = task_directory / "radar-center.json"
            launcher = MissionScreenLauncher(
                task_directory, 10.0, config_path=config_path
            )
            launcher.child = unittest.mock.Mock()
            launcher.child.poll.return_value = None

            launcher.receive(b"36.5\xff\xff\xff", 1.0)

            self.assertEqual(20.0, launcher.radar_center_behind_a_cm)
            self.assertFalse(config_path.exists())

    def test_distance_digits_inside_a_larger_number_are_ignored(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            task_directory = Path(directory)
            launcher = MissionScreenLauncher(task_directory, 10.0)

            launcher.receive(b"120\xff", 1.0)

            self.assertEqual(20.0, launcher.radar_center_behind_a_cm)
            self.assertFalse((task_directory / "radar_center_config.json").exists())

    @patch("mission_screen_launcher.Path.is_file", return_value=True)
    @patch("mission_screen_launcher.subprocess.Popen")
    def test_saved_distance_is_injected_into_real_task_command(
        self, popen, _is_file
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            task_directory = Path(directory)
            launcher = MissionScreenLauncher(task_directory, 10.0)
            launcher.receive(b"36.5\xff\xff\xff", 1.0)
            launcher.schedule(task_directory / "main_task1.py", 2.0, delay_s=0.0)

            process = popen.return_value
            process.poll.return_value = None
            process.pid = 42
            launcher.poll(2.0)

            popen.assert_called_once_with(
                [
                    sys.executable,
                    str(task_directory / "main_task1.py"),
                    "--radar-center-behind-a-cm",
                    "36.5",
                ],
                cwd=task_directory,
                start_new_session=True,
            )


if __name__ == "__main__":
    unittest.main()
