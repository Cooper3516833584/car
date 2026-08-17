import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from components import TrackSegment
from components.radar_camera_line_following import (
    MainConfig,
    RadarCameraLineApplication,
)


class FakeAlarm:
    def __init__(self):
        self.off_calls = 0
        self.on_calls = 0
        self.is_initialized = True

    def on(self):
        self.on_calls += 1

    def off(self):
        self.off_calls += 1


class FakeStopEvent:
    def __init__(self):
        self.waits = []

    def wait(self, duration):
        self.waits.append(duration)
        return False


class Mission1CompletionAlarmTests(unittest.TestCase):
    def test_completed_edge_starts_alarm_once(self):
        app = object.__new__(RadarCameraLineApplication)
        app.camera_corrector = SimpleNamespace(
            set_curve_mode=lambda _enabled: None
        )
        app._lock = threading.Lock()
        app._follower_state = SimpleNamespace(completed=False)
        app._final_da_visual_error_cm = None
        app._final_da_visual_timestamp_s = None
        app._terminal_camera_disagreement = False
        app.radar = SimpleNamespace(set_motion_hint=lambda _active: None)
        app._completed_event = threading.Event()
        app.follower = SimpleNamespace(terminal_hard_stop_triggered=False)
        app.config = SimpleNamespace(
            radar_center_behind_a_cm=20.0,
            completion_alarm_seconds=1.0,
            mission_control=MainConfig().mission_control,
        )
        app._start_completion_alarm = Mock()
        completed = SimpleNamespace(
            completed=True,
            segment=TrackSegment.DA,
        )

        app._on_follower_state(completed)
        app._on_follower_state(completed)

        app._start_completion_alarm.assert_called_once_with()
        self.assertTrue(app._completed_event.is_set())

    def test_alarm_waits_one_second_and_always_turns_off(self):
        app = object.__new__(RadarCameraLineApplication)
        app.config = MainConfig(completion_alarm_seconds=1.0)
        app._stop_event = FakeStopEvent()
        app._completion_alarm_lock = threading.Lock()
        app._completion_alarm_device = None
        alarm = FakeAlarm()

        with patch.object(app, "_build_alarm", return_value=alarm) as build:
            app._run_completion_alarm()

        self.assertEqual([1.0], app._stop_event.waits)
        self.assertEqual(1, alarm.on_calls)
        self.assertEqual(1, alarm.off_calls)
        build.assert_called_once_with()
        self.assertIsNone(app._completion_alarm_device)

    def test_alarm_start_failure_is_nonfatal_and_uses_off_fallback(self):
        app = object.__new__(RadarCameraLineApplication)
        app.config = MainConfig(completion_alarm_seconds=1.0)
        app._stop_event = FakeStopEvent()
        app._completion_alarm_lock = threading.Lock()
        app._completion_alarm_device = None

        with patch.object(
            app,
            "_build_alarm",
            side_effect=RuntimeError("GPIO unavailable"),
        ) as build:
            # Must not propagate; the fallback silence attempt may also fail.
            app._run_completion_alarm()

        self.assertEqual(2, build.call_count)
        self.assertIsNone(app._completion_alarm_device)

    def test_start_is_deduplicated_and_uses_named_worker(self):
        app = object.__new__(RadarCameraLineApplication)
        app._completion_alarm_lock = threading.Lock()
        app._completion_alarm_started = False
        app._completion_alarm_thread = None
        ran = threading.Event()
        app._run_completion_alarm = ran.set

        app._start_completion_alarm()
        thread = app._completion_alarm_thread
        app._start_completion_alarm()
        thread.join(timeout=1.0)

        self.assertTrue(ran.is_set())
        self.assertEqual("mission1-car-completion-alarm", thread.name)
        self.assertFalse(thread.daemon)

    def test_negative_completion_alarm_is_rejected(self):
        with self.assertRaises(ValueError):
            MainConfig(completion_alarm_seconds=-0.1)

    def test_close_joins_alarm_before_fleet_resources(self):
        calls = []

        class Thread:
            def join(self, timeout):
                calls.append(("join", timeout))

        class Closeable:
            def __init__(self, name):
                self.name = name

            def close(self):
                calls.append((self.name,))

        class StopEvent:
            def set(self):
                calls.append(("stop",))

        app = object.__new__(RadarCameraLineApplication)
        app._lock = threading.Lock()
        app._closed = False
        app._ready = True
        app._map_ready = True
        app._calibrating = False
        app._stop_event = StopEvent()
        app._completion_alarm_lock = threading.Lock()
        app._completion_alarm_thread = Thread()
        app._completion_alarm_device = FakeAlarm()
        app.config = SimpleNamespace(completion_alarm_seconds=1.0)
        app.fleet_node = Closeable("fleet-node")
        app.fleet_link = Closeable("fleet-link")
        app._alarm = None
        app.camera_corrector = Closeable("camera")
        app.follower = SimpleNamespace(
            stop_mission=lambda: calls.append(("follower",))
        )
        app.radar = SimpleNamespace(
            set_motion_hint=lambda _active: calls.append(("radar-hint",)),
            close=lambda: calls.append(("radar",)),
        )
        app.drive = Closeable("drive")

        app.close()

        self.assertLess(
            calls.index(("join", 1.5)),
            calls.index(("fleet-node",)),
        )
        self.assertEqual(1, app._completion_alarm_device.off_calls)


if __name__ == "__main__":
    unittest.main()
