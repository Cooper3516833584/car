import time
import threading
import unittest
from types import SimpleNamespace

from components.navigation import NavigationPose, NavigationState
from main import CompetitionCarApplication
from fleet_bus.command_queue import CarCommandQueue
from fleet_bus.models import (
    AckStatus,
    CarNavigateCommand,
    CommandId,
    NodeFlags,
    ReportPayload,
)
from fleet_bus.pose_provider import CarFleetStateProvider
from fleet_bus.protocol import (
    decode_car_navigate,
    decode_report,
    encode_car_navigate,
    encode_report,
)


class FleetBusProtocolTests(unittest.TestCase):
    def test_report_and_navigation_payload_round_trip(self):
        report = ReportPayload(
            1, 2, 3, 4, -5, 6, 0, 35999, 8, -9, 0, 1200, 4, 4, 0, 0, 0
        )
        self.assertEqual(report, decode_report(encode_report(report)))
        command = CarNavigateCommand(20, -30, 35999)
        self.assertEqual(command, decode_car_navigate(encode_car_navigate(command)))


class CarCommandQueueTests(unittest.TestCase):
    def test_stop_precedes_navigation_and_status_is_report_only(self):
        queue = CarCommandQueue(maxsize=2)
        navigate = SimpleNamespace(seq=3, command_id=CommandId.CAR_NAVIGATE_TO)
        stop = SimpleNamespace(seq=4, command_id=CommandId.TARGETED_STOP)
        self.assertTrue(queue.put(navigate))
        self.assertTrue(queue.put(stop))
        self.assertIs(stop, queue.receive(timeout=0))
        queue.accept(stop)
        self.assertEqual(4, queue.status().active_command_seq)
        self.assertEqual(int(AckStatus.ACCEPTED), queue.status().status)
        queue.complete(stop)
        self.assertEqual(int(AckStatus.COMPLETED), queue.status().status)


class CarFleetStateProviderTests(unittest.TestCase):
    def test_application_runtime_snapshot_is_read_only_and_local(self):
        application = object.__new__(CompetitionCarApplication)
        application._lock = threading.Lock()
        application._ready = True
        application._map_ready = True
        application._latest_navigation_pose = NavigationPose(5, -6, 90)
        application._follower_state = SimpleNamespace(
            completed=False, running=True
        )
        application._localization_degraded = False
        application._fleet_error_code = 0
        snapshot = application.fleet_runtime_snapshot()
        self.assertEqual((5, -6, 90), (
            snapshot.pose.x_cm, snapshot.pose.y_cm, snapshot.pose.heading_deg
        ))
        self.assertIs(NavigationState.FOLLOWING, snapshot.navigation_state)

    def test_fresh_local_pose_is_reported_without_field_transform(self):
        now = time.monotonic()
        snapshot = SimpleNamespace(
            ready=True,
            map_ready=True,
            pose=NavigationPose(12.4, -8.6, 359.99, now),
            navigation_state=NavigationState.FOLLOWING,
            localization_degraded=False,
            error_code=0,
            localization_timeout_s=0.5,
        )
        state = CarFleetStateProvider(lambda: snapshot, started_at=now)()
        self.assertEqual(
            (12, -9, 35999), (state.x_cm, state.y_cm, state.heading_cdeg)
        )
        self.assertTrue(state.node_flags & int(NodeFlags.READY))
        self.assertTrue(state.node_flags & int(NodeFlags.MAP_READY))
        self.assertTrue(state.node_flags & int(NodeFlags.POSE_VALID))
        self.assertTrue(state.node_flags & int(NodeFlags.BUSY))
        self.assertEqual(4, state.pose_quality)

    def test_stale_pose_is_degraded_but_keeps_recent_local_value(self):
        snapshot = SimpleNamespace(
            ready=True,
            map_ready=True,
            pose=NavigationPose(1, 2, 3, time.monotonic() - 2),
            navigation_state=NavigationState.IDLE,
            localization_degraded=False,
            error_code=7,
            localization_timeout_s=0.5,
        )
        state = CarFleetStateProvider(lambda: snapshot)()
        self.assertFalse(state.node_flags & int(NodeFlags.POSE_VALID))
        self.assertTrue(state.node_flags & int(NodeFlags.LOCALIZATION_DEGRADED))
        self.assertEqual((1, 2, 300), (state.x_cm, state.y_cm, state.heading_cdeg))
        self.assertEqual(2, state.pose_quality)
        self.assertEqual(7, state.error_code)


if __name__ == "__main__":
    unittest.main()
