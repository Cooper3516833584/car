from __future__ import annotations

import shutil
import sys
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mission_screen_launcher import MissionScreenLauncher  # noqa: E402
from config.loader import load_car_config  # noqa: E402
from config.runtime_state import load_runtime_radar_center_cm  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
REAL_CONFIG = REPO_ROOT / "configs" / "cooper_rock5a_l150.toml"


class MissionScreenLauncherTests(unittest.TestCase):
    def setUp(self) -> None:
        # Use Path.mkdir (not tempfile.mkdtemp): the Windows file sandbox
        # denies creating subdirectories inside mkdtemp-created directories.
        self.temporary_directory = (
            Path(__file__).resolve().parent
            / f"_launcher_tmp_{uuid.uuid4().hex}"
        )
        self.temporary_directory.mkdir()
        self.addCleanup(
            shutil.rmtree, self.temporary_directory, ignore_errors=True
        )
        # Mirrors the deployment layout: the launcher runs from a "code" task
        # directory and the runtime state is anchored at its parent.
        self.task_directory = self.temporary_directory / "code"
        self.task_directory.mkdir()

    def test_idle_reporter_restarts_around_child_task(self) -> None:
        events = []

        class Reporter:
            def __init__(self, distance_provider, port):
                self.distance_provider = distance_provider
                self.port = port

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

        launcher.poll(1.0)

        self.assertEqual(3, alarm_class.return_value.on.call_count)
        self.assertEqual(
            [
                sys.executable,
                str(Path("/tasks/main_task2.py")),
                "--config",
                str(launcher.config_path),
                "--radar-center-behind-a-cm",
                "20",
            ],
            popen.call_args.args[0],
        )

    def test_fragmented_radar_distance_is_saved_and_passed_to_task(self) -> None:
        config_path = REAL_CONFIG
        launcher = MissionScreenLauncher(
            self.task_directory, 10.0, config_path=config_path
        )

        launcher.receive(b"36", 1.0)
        launcher.receive(b".5\xff\xff\xff", 2.0)

        self.assertEqual(36.5, launcher.radar_center_behind_a_cm)
        state_file = (
            self.temporary_directory
            / "runtime"
            / "car_state.json"
        )
        self.assertEqual(
            36.5,
            load_runtime_radar_center_cm(
                launcher.car_config.runtime,
                20.0,
                base_directory=self.temporary_directory,
            ),
        )
        self.assertTrue(state_file.exists())
        self.assertIsNone(launcher.pending)

    def test_saved_radar_distance_survives_launcher_restart(self) -> None:
        config_path = REAL_CONFIG
        first = MissionScreenLauncher(
            self.task_directory, 10.0, config_path=config_path
        )
        first.receive(b"36.5", 1.0)

        second = MissionScreenLauncher(
            self.task_directory, 10.0, config_path=config_path
        )

        self.assertEqual(36.5, second.radar_center_behind_a_cm)

    def test_distance_change_is_ignored_while_task_is_running(self) -> None:
        launcher = MissionScreenLauncher(
            self.task_directory, 10.0, config_path=REAL_CONFIG
        )
        launcher.child = unittest.mock.Mock()
        launcher.child.poll.return_value = None

        launcher.receive(b"36.5\xff\xff\xff", 1.0)

        self.assertEqual(20.0, launcher.radar_center_behind_a_cm)
        self.assertFalse(
            (self.temporary_directory / "runtime" / "car_state.json").exists()
        )

    def test_distance_digits_inside_a_larger_number_are_ignored(self) -> None:
        launcher = MissionScreenLauncher(
            self.task_directory, 10.0, config_path=REAL_CONFIG
        )

        launcher.receive(b"120\xff", 1.0)

        self.assertEqual(20.0, launcher.radar_center_behind_a_cm)
        self.assertFalse(
            (self.temporary_directory / "runtime" / "car_state.json").exists()
        )

    @patch("mission_screen_launcher.Path.is_file", return_value=True)
    @patch("mission_screen_launcher.subprocess.Popen")
    def test_saved_distance_is_injected_into_real_task_command(
        self, popen, _is_file
    ) -> None:
        launcher = MissionScreenLauncher(
            self.task_directory, 10.0, config_path=REAL_CONFIG
        )
        launcher.receive(b"36.5\xff\xff\xff", 1.0)
        launcher.schedule(
            self.task_directory / "main_task1.py", 2.0, delay_s=0.0
        )

        process = popen.return_value
        process.poll.return_value = None
        process.pid = 42
        launcher.poll(2.0)

        self.assertEqual(
            [
                sys.executable,
                str(self.task_directory / "main_task1.py"),
                "--config",
                str(REAL_CONFIG),
                "--radar-center-behind-a-cm",
                "36.5",
            ],
            popen.call_args.args[0],
        )

    def test_launcher_reads_screen_and_hc14_from_profile(self) -> None:
        launcher = MissionScreenLauncher(
            self.task_directory, 10.0, config_path=REAL_CONFIG
        )
        profile = load_car_config(REAL_CONFIG)
        self.assertEqual(
            profile.devices.screen.port, launcher.screen_port
        )
        self.assertEqual(
            profile.devices.screen.baudrate, launcher.screen_baudrate
        )
        self.assertEqual(profile.devices.hc14.port, launcher.hc14_port)

    def test_screen_disabled_does_not_open_serial(self) -> None:
        import mission_screen_launcher as launcher_module

        content = REAL_CONFIG.read_text(encoding="utf-8")
        content = content.replace(
            "[devices.screen]\nenabled = true",
            "[devices.screen]\nenabled = false",
        )
        config_path = self.temporary_directory / "screen-disabled.toml"
        config_path.write_text(content, encoding="utf-8")

        with patch.object(
            launcher_module, "configure_serial"
        ) as configure_serial:
            code = launcher_module.main(["--config", str(config_path)])

        self.assertEqual(0, code)
        configure_serial.assert_not_called()


if __name__ == "__main__":
    unittest.main()
