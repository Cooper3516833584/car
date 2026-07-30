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
    def test_later_button_replaces_pending_choice(
        self, _alarm_class, _sleep
    ) -> None:
        launcher = MissionScreenLauncher(Path("/tasks"), 10.0)
        launcher.receive(b"MISSION1", 1.0)
        launcher.receive(b"MISSION2", 3.0)
        assert launcher.pending is not None
        self.assertEqual(launcher.pending.task_path.name, "main_task2.py")
        self.assertEqual(launcher.pending.due_at, 13.0)

    @patch("mission_screen_launcher.Path.is_file", return_value=True)
    @patch("mission_screen_launcher.subprocess.Popen")
    @patch("mission_screen_launcher.SoundLightAlarm")
    def test_sounds_alarm_for_final_five_seconds_then_launches(
        self, alarm_class, popen, _is_file
    ) -> None:
        process = popen.return_value
        process.poll.return_value = None
        process.pid = 42
        launcher = MissionScreenLauncher(Path("/tasks"), 10.0)
        launcher.receive(b"MISSION2", 1.0)
        launcher.poll(5.9)
        popen.assert_not_called()
        launcher.poll(6.0)
        alarm_class.return_value.on.assert_called_once()
        launcher.poll(10.9)
        popen.assert_not_called()
        launcher.poll(11.0)
        alarm_class.return_value.off.assert_called_once()
        popen.assert_called_once_with(
            [sys.executable, str(Path("/tasks/main_task2.py"))],
            cwd=Path("/tasks"), start_new_session=True,
        )


if __name__ == "__main__":
    unittest.main()
