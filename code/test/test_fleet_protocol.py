import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components.fleet_models import *  # noqa: E402,F401,F403
from components.fleet_protocol import *  # noqa: E402,F401,F403

DATA = json.loads((Path(__file__).parent / "data" / "fleetbus_v1_golden.json").read_text())


class FleetProtocolTests(unittest.TestCase):
    def test_v1_wire_contract_constants(self):
        self.assertEqual(MAGIC, b"\xd3\x91")
        self.assertEqual(TAIL, b"\x1d\x0f")
        self.assertEqual(VERSION, 1)
        self.assertEqual(MAX_PAYLOAD_LEN, 220)
        self.assertEqual(HEADER.format, "<2sBBBBBIHH")
        self.assertEqual(REPORT.format, "<IHHIiiiHhhhHBBHBB")

    def test_payload_vectors_match_golden(self):
        terrain = tuple(range(1, 8)) + tuple(range(1, 8)) + (1,)
        positions = tuple(
            (115 + 70 * col, 175 + 70 * row)
            for row in range(3)
            for col in range(5)
        )
        encoded = {
            "trace_request": encode_trace_request(
                TraceRequestPayload(0x12345678, 100, 15, 0)
            ).hex(),
            "trace_report": encode_trace_report(
                TraceReportPayload(
                    0x10203040,
                    0x5060,
                    0x708090A0,
                    7,
                    8,
                    10,
                    0,
                    (
                        TraceSample(1000, -100, 200, -300, 35999, 4, 1),
                        TraceSample(1100, -90, 180, -270, 0, 3, 3),
                        TraceSample(1200, 10, 80, -170, 1234, 2, 1),
                    ),
                )
            ).hex(),
            "poll": encode_poll(PollPayload(7)).hex(),
            "report": encode_report(
                ReportPayload(1, 2, 3, 4, -5, 6, 7, 800, 9, -10, 11, 1200, 4, 3, 12, 2, 0)
            ).hex(),
            "ack": encode_ack(
                AckPayload(1, 2, CommandId.PING, AckStatus.COMPLETED, AckReason.NONE, "ok")
            ).hex(),
            "coordinate_frame": encode_coordinate_frame(
                CoordinateFrameCommand(10, -20, 35999)
            ).hex(),
            "car_navigate": encode_car_navigate(
                CarNavigateCommand(100, -50, 9000)
            ).hex(),
            "drone_goto": encode_drone_goto(
                DroneGotoCommand(100, -50, 120, None)
            ).hex(),
            "map_report": encode_map_report(
                MapReportPayload(1, 2, 3, ((0, 0), (1, 0), (1, 1), (0, 1)))
            ).hex(),
            "path_report": encode_path_report(
                PathReportPayload(1, 2, 3, ((0, 0), (5, 6)))
            ).hex(),
            "survey_report": encode_survey_report(
                SurveyReportPayload(
                    1, 2, 3, int(SurveyFlags.COMPLETE), 4, 2, 3, 5, 1, 4, terrain
                )
            ).hex(),
            "survey_report_positions": encode_survey_report(
                SurveyReportPayload(
                    1, 2, 4,
                    int(SurveyFlags.COMPLETE | SurveyFlags.ABSOLUTE_POSITIONS),
                    4, 2, 3, 5, 1, 4, terrain, positions,
                )
            ).hex(),
            "disaster_rescue": encode_disaster_rescue(
                DisasterRescueCommand(4, 2, 3, terrain)
            ).hex(),
        }
        self.assertEqual(
            "78563412640000000f00",
            encoded.pop("trace_request"),
        )
        self.assertEqual(
            "403020106050a090807007000000080000000a0000000300e80300009cffffffc8000000d4feffff9f8c040164000a00ecff1e0000000303640064009cff6400d2040201",
            encoded.pop("trace_report"),
        )
        self.assertEqual(set(encoded), set(DATA["payload_vectors"]))
        for name, expected in DATA["payload_vectors"].items():
            self.assertEqual(encoded[name], expected, name)

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
        terrain = tuple(range(1, 8)) + tuple(range(1, 8)) + (1,)
        survey = SurveyReportPayload(1, 2, 3, int(SurveyFlags.COMPLETE), 4, 2, 3, 5, 1, 4, terrain)
        self.assertEqual(decode_survey_report(encode_survey_report(survey)), survey)
        positions = tuple((115 + 70 * col, 175 + 70 * row) for row in range(3) for col in range(5))
        extended = SurveyReportPayload(
            1, 2, 4,
            int(SurveyFlags.COMPLETE | SurveyFlags.ABSOLUTE_POSITIONS),
            4, 2, 3, 5, 1, 4, terrain, positions,
        )
        self.assertEqual(decode_survey_report(encode_survey_report(extended)), extended)
        rescue = DisasterRescueCommand(4, 2, 3, terrain)
        self.assertEqual(decode_disaster_rescue(encode_disaster_rescue(rescue)), rescue)
        counter = SequenceCounter(0xFFFE)
        self.assertEqual((counter.next(), counter.next()), (0xFFFF, 1))
        cache = RecentResponseCache()
        cache.put(10, 1, b"a")
        cache.begin_ground_session(11)
        self.assertIsNone(cache.get(10, 1))


if __name__ == "__main__":
    unittest.main()
