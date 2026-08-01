from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mission_screen_launcher import MissionScreenLauncher  # noqa: E402


class MissionScreenLauncherTests(unittest.TestCase):
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
            [sys.executable, str(Path("/tasks/main_task2.py"))],
            cwd=Path("/tasks"), start_new_session=True,
        )


if __name__ == "__main__":
    unittest.main()
