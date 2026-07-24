from pathlib import Path
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components.fleet_car_node import FleetCarNode  # noqa: E402
from components.fleet_models import *  # noqa: E402,F401,F403
from components.fleet_protocol import *  # noqa: E402,F401,F403


class Harness:
    def __init__(self):
        self.writes = []
        self.coordinate_calls = []
        self.navigate_calls = []
        self.stop_calls = 0
        self.callback_threads = []
        self.event = threading.Event()
        self.state = CarFleetState(
            int(NodeFlags.POSE_VALID | NodeFlags.READY | NodeFlags.MAP_READY),
            1000, 10, 20, 9000, pose_quality=3,
            map_revision=2,
            field_corners=((0, 0), (100, 0), (100, 100), (0, 100)),
            path_revision=3,
            path_points=((0, 0), (50, 50), (100, 100)),
        )
        self.node = FleetCarNode(
            writer=self.write,
            state_provider=lambda: self.state,
            on_set_coordinate_frame=self.coordinate,
            on_navigate=self.navigate,
            on_stop=self.stop,
            timing=NodeTiming(0, 16),
            wait=lambda _: False,
        )
        self.node.start()

    def write(self, raw):
        self.writes.append(raw)
        self.event.set()

    def coordinate(self, value):
        self.callback_threads.append(threading.get_ident())
        self.coordinate_calls.append(value)
        return CommandResult(AckStatus.COMPLETED)

    def navigate(self, value):
        self.callback_threads.append(threading.get_ident())
        self.navigate_calls.append(value)
        return CommandResult(AckStatus.ACCEPTED)

    def stop(self):
        self.stop_calls += 1
        return CommandResult(AckStatus.COMPLETED)

    def send(self, frame):
        self.event.clear()
        self.node.feed_frame(pack_frame(frame))
        self.assert_reply()
        return self.writes[-1]

    def assert_reply(self):
        if not self.event.wait(0.5):
            raise AssertionError("node did not reply")


class FleetCarNodeTests(unittest.TestCase):
    def setUp(self):
        self.h = Harness()
        self.addCleanup(self.h.node.close)
        self.ground_session = 55

    def request(self, kind, payload, seq=1, dst=NodeId.CAR, session=None):
        return Frame(
            VERSION, NodeId.GROUND, dst, kind, 0,
            self.ground_session if session is None else session, seq, payload,
        )

    def test_non_car_address_is_silent(self):
        self.h.node.feed_frame(pack_frame(self.request(MessageKind.POLL, b"", dst=NodeId.DRONE)))
        self.assertFalse(self.h.event.wait(0.05))

    def test_poll_reports_car_state(self):
        raw = self.h.send(self.request(MessageKind.POLL, encode_poll(PollPayload())))
        frame = unpack_frame(raw)
        report = decode_report(frame.payload)
        self.assertEqual((report.x_cm, report.y_cm, report.z_cm), (10, 20, 0))
        self.assertEqual((report.request_session, report.request_seq), (55, 1))

    def test_duplicate_coordinate_command_executes_once_and_ack_is_identical(self):
        body = encode_coordinate_frame(CoordinateFrameCommand(1, 2, 300))
        request = self.request(
            MessageKind.COMMAND,
            encode_command(CommandPayload(CommandId.SET_COORDINATE_FRAME, 0, body)),
        )
        first = self.h.send(request)
        second = self.h.send(request)
        self.assertEqual(first, second)
        self.assertEqual(len(self.h.coordinate_calls), 1)

    def test_navigation_runs_on_worker_not_rx_caller(self):
        caller = threading.get_ident()
        body = encode_car_navigate(CarNavigateCommand(50, 60, 700))
        raw = self.h.send(self.request(
            MessageKind.COMMAND,
            encode_command(CommandPayload(CommandId.CAR_NAVIGATE_TO, 0, body)),
        ))
        self.assertNotEqual(self.h.callback_threads[-1], caller)
        self.assertEqual(decode_ack(unpack_frame(raw).payload).status, AckStatus.ACCEPTED)

    def test_ground_session_change_clears_dedupe(self):
        request1 = self.request(
            MessageKind.COMMAND,
            encode_command(CommandPayload(CommandId.TARGETED_STOP)),
            session=100,
        )
        request2 = self.request(
            MessageKind.COMMAND,
            encode_command(CommandPayload(CommandId.TARGETED_STOP)),
            session=101,
        )
        self.h.send(request1)
        self.h.send(request1)
        self.h.send(request2)
        self.assertEqual(self.h.stop_calls, 2)

    def test_map_and_path_are_bounded_reports(self):
        map_frame = unpack_frame(self.h.send(self.request(MessageKind.MAP_REQUEST, b"")))
        self.assertEqual(len(decode_map_report(map_frame.payload).corners), 4)
        path_frame = unpack_frame(self.h.send(self.request(MessageKind.PATH_REQUEST, b"", seq=2)))
        self.assertEqual(len(decode_path_report(path_frame.payload).points), 3)

    def test_stop_is_idempotent_for_duplicate_and_new_sequence(self):
        payload = encode_command(CommandPayload(CommandId.TARGETED_STOP))
        self.h.send(self.request(MessageKind.COMMAND, payload, seq=4))
        self.h.send(self.request(MessageKind.COMMAND, payload, seq=4))
        self.h.send(self.request(MessageKind.COMMAND, payload, seq=5))
        self.assertEqual(self.h.stop_calls, 2)

    def test_close_prevents_late_write(self):
        self.h.node.close()
        count = len(self.h.writes)
        self.h.node.feed_frame(pack_frame(self.request(MessageKind.POLL, b"")))
        time.sleep(0.05)
        self.assertEqual(len(self.h.writes), count)


if __name__ == "__main__":
    unittest.main()
