import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components.fleet_models import *  # noqa: E402,F401,F403
from components.fleet_protocol import *  # noqa: E402,F401,F403

DATA = json.loads((Path(__file__).parent / "data" / "fleetbus_v1_golden.json").read_text())


class FleetProtocolTests(unittest.TestCase):
    def test_crc_and_golden_frames(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)
        for item in DATA["valid_frames"]:
            raw = bytes.fromhex(item["frame_hex"])
            self.assertEqual(pack_frame(unpack_frame(raw)), raw, item["name"])

    def test_fragmentation_sticky_bad_crc_and_address(self):
        raw = bytes.fromhex(DATA["scenarios"]["fragmentation_hex"])
        parser, frames = FrameParser(), []
        for byte in raw:
            frames.extend(parser.feed(bytes((byte,))))
        self.assertEqual(len(frames), 1)
        self.assertIn(MAGIC + TAIL, frames[0].payload)
        self.assertEqual(len(FrameParser().feed(bytes.fromhex(DATA["scenarios"]["sticky_hex"]))), 2)
        bad = bytes.fromhex(DATA["scenarios"]["bad_crc_hex"])
        good = bytes.fromhex(DATA["valid_frames"][0]["frame_hex"])
        parser = FrameParser()
        self.assertEqual(parser.feed(bad + good), [unpack_frame(good)])
        self.assertEqual(parser.stats.crc_failures, 1)
        parser = FrameParser(local_node=NodeId.CAR)
        parser.feed(good)
        self.assertEqual(parser.stats.address_drops, 1)

    def test_payload_codecs_sequence_and_cache(self):
        report = ReportPayload(1, 2, 3, 4, -5, 6, 7, 800, 9, -10, 11, 1200, 4, 3, 12, 2, 0)
        self.assertEqual(decode_report(encode_report(report)), report)
        ack = AckPayload(1, 2, CommandId.PING, AckStatus.COMPLETED, AckReason.NONE, "ok")
        self.assertEqual(decode_ack(encode_ack(ack)), ack)
        coordinate = CoordinateFrameCommand(10, -20, 35999)
        self.assertEqual(decode_coordinate_frame(encode_coordinate_frame(coordinate)), coordinate)
        car = CarNavigateCommand(100, -50, 9000)
        self.assertEqual(decode_car_navigate(encode_car_navigate(car)), car)
        drone = DroneGotoCommand(100, -50, 120, None)
        self.assertEqual(decode_drone_goto(encode_drone_goto(drone)), drone)
        map_value = MapReportPayload(1, 2, 3, ((0, 0), (1, 0), (1, 1), (0, 1)))
        self.assertEqual(decode_map_report(encode_map_report(map_value)), map_value)
        path = PathReportPayload(1, 2, 3, ((0, 0), (5, 6)))
        self.assertEqual(decode_path_report(encode_path_report(path)), path)
        counter = SequenceCounter(0xFFFE)
        self.assertEqual((counter.next(), counter.next()), (0xFFFF, 1))
        cache = RecentResponseCache()
        cache.put(10, 1, b"a")
        cache.begin_ground_session(11)
        self.assertIsNone(cache.get(10, 1))


if __name__ == "__main__":
    unittest.main()
