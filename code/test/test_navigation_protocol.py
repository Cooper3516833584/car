"""Hardware-free tests for authenticated coordinate navigation commands."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.navigation import NavigationGoal  # noqa: E402
from components.navigation_protocol import (  # noqa: E402
    COMMAND_STOP_MISSION,
    AckStatus,
    GroundNavigationProtocol,
    NavigationProtocolError,
    decode_navigation_payload,
    encode_navigation_payload,
    pack_authenticated_frame,
    pack_navigation_command,
    unpack_authenticated_frame,
)


KEY = bytes.fromhex("00112233445566778899aabbccddeeff")


class NavigationPayloadTests(unittest.TestCase):
    def test_goal_without_heading_round_trip(self) -> None:
        decoded = decode_navigation_payload(encode_navigation_payload(NavigationGoal(120, -35)))
        self.assertEqual(decoded.x_cm, 120)
        self.assertEqual(decoded.y_cm, -35)
        self.assertIsNone(decoded.final_heading_deg)

    def test_goal_with_heading_round_trip(self) -> None:
        decoded = decode_navigation_payload(
            encode_navigation_payload(NavigationGoal(120, -35, 359.25))
        )
        self.assertEqual(decoded.final_heading_deg, 359.25)

    def test_authenticated_frame_rejects_tampering(self) -> None:
        frame = bytearray(
            pack_navigation_command(
                NavigationGoal(100, 200, 90), session=3, seq=7, key=KEY
            )
        )
        frame[15] ^= 1
        with self.assertRaises(NavigationProtocolError):
            unpack_authenticated_frame(bytes(frame), key=KEY)


class GroundNavigationProtocolTests(unittest.TestCase):
    def test_valid_command_is_called_once_and_duplicate_is_idempotent(self) -> None:
        goals = []
        protocol = GroundNavigationProtocol(
            key=KEY,
            on_goal=lambda goal, receipt: goals.append((goal, receipt)),
            on_stop=lambda receipt: None,
        )
        frame = pack_navigation_command(
            NavigationGoal(300, -100, 45), session=9, seq=11, key=KEY
        )
        first = protocol.handle_frame(frame)
        second = protocol.handle_frame(frame)
        self.assertEqual(first, second)
        self.assertEqual(len(goals), 1)
        self.assertEqual(len(first), 2)
        ack = unpack_authenticated_frame(first[-1], key=KEY)
        self.assertEqual(ack.payload[-2], AckStatus.ACCEPTED)

    def test_existing_stop_mission_command_is_supported(self) -> None:
        stops = []
        protocol = GroundNavigationProtocol(
            key=KEY,
            on_goal=lambda goal, receipt: None,
            on_stop=stops.append,
        )
        frame = pack_authenticated_frame(
            4,
            bytes((COMMAND_STOP_MISSION,)),
            session=4,
            seq=5,
            key=KEY,
        )
        replies = protocol.handle_frame(frame)
        self.assertEqual(len(stops), 1)
        self.assertEqual(len(replies), 2)


if __name__ == "__main__":
    unittest.main()
