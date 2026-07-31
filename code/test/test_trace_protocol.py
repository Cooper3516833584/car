"""Hardware-free tests for the FleetBus trace payload codecs."""

from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.fleet_models import (  # noqa: E402
    TraceReportFlags,
    TraceReportPayload,
    TraceRequestPayload,
    TraceSample,
)
from components.fleet_protocol import (  # noqa: E402
    MAX_PAYLOAD_LEN,
    ProtocolError,
    TRACE_MAX_SAMPLES,
    TRACE_REPORT_HEADER,
    TRACE_SAMPLE_ABSOLUTE,
    TRACE_SAMPLE_DELTA,
    decode_trace_report,
    decode_trace_request,
    encode_trace_report,
    encode_trace_request,
)


GOLDEN_TRACE_REQUEST_HEX = "78563412640000000f00"
GOLDEN_TRACE_REPORT_HEX = (
    "403020106050a090807007000000080000000a0000000300"
    "e80300009cffffffc8000000d4feffff9f8c0401"
    "64000a00ecff1e0000000303"
    "640064009cff6400d2040201"
)


def make_sample(
    uptime_ms=1000,
    x_cm=10,
    y_cm=20,
    z_cm=30,
    heading_cdeg=9000,
    quality=4,
    flags=1,
):
    return TraceSample(
        uptime_ms=uptime_ms,
        x_cm=x_cm,
        y_cm=y_cm,
        z_cm=z_cm,
        heading_cdeg=heading_cdeg,
        quality=quality,
        flags=flags,
    )


def make_report(samples=(), **changes):
    samples = tuple(samples)
    values = {
        "request_session": 0x10203040,
        "request_seq": 0x5060,
        "trace_session": 0x708090A0,
        "oldest_available_seq": 1 if samples else 0,
        "first_sample_seq": 1 if samples else 0,
        "latest_available_seq": len(samples),
        "report_flags": int(TraceReportFlags.NONE),
        "samples": samples,
    }
    values.update(changes)
    return TraceReportPayload(**values)


def golden_report():
    return TraceReportPayload(
        request_session=0x10203040,
        request_seq=0x5060,
        trace_session=0x708090A0,
        oldest_available_seq=7,
        first_sample_seq=8,
        latest_available_seq=10,
        report_flags=int(TraceReportFlags.NONE),
        samples=(
            TraceSample(1000, -100, 200, -300, 35999, 4, 1),
            TraceSample(1100, -90, 180, -270, 0, 3, 3),
            TraceSample(1200, 10, 80, -170, 1234, 2, 1),
        ),
    )


class TraceProtocolTests(unittest.TestCase):
    def test_trace_request_round_trip_and_golden_bytes(self):
        request = TraceRequestPayload(0x12345678, 100, 15, 0)
        encoded = encode_trace_request(request)
        self.assertEqual(encoded.hex(), GOLDEN_TRACE_REQUEST_HEX)
        self.assertEqual(decode_trace_request(encoded), request)

    def test_trace_request_accepts_maximum_sample_count(self):
        request = TraceRequestPayload(1, 2, TRACE_MAX_SAMPLES, 0)
        self.assertEqual(decode_trace_request(encode_trace_request(request)), request)

    def test_trace_request_rejects_zero_sample_count(self):
        with self.assertRaises(ProtocolError):
            encode_trace_request(TraceRequestPayload(1, 2, 0, 0))
        with self.assertRaises(ProtocolError):
            decode_trace_request(bytes.fromhex("01000000020000000000"))

    def test_trace_request_rejects_sixteen_samples(self):
        with self.assertRaises(ProtocolError):
            encode_trace_request(TraceRequestPayload(1, 2, 16, 0))
        with self.assertRaises(ProtocolError):
            decode_trace_request(bytes.fromhex("01000000020000001000"))

    def test_trace_request_rejects_nonzero_flags(self):
        with self.assertRaises(ProtocolError):
            encode_trace_request(TraceRequestPayload(1, 2, 1, 1))
        with self.assertRaises(ProtocolError):
            decode_trace_request(bytes.fromhex("01000000020000000101"))

    def test_empty_trace_report_round_trip(self):
        report = make_report()
        encoded = encode_trace_report(report)
        self.assertEqual(len(encoded), TRACE_REPORT_HEADER.size)
        self.assertEqual(decode_trace_report(encoded), report)

    def test_single_trace_sample_uses_absolute_encoding(self):
        report = make_report((make_sample(),))
        encoded = encode_trace_report(report)
        self.assertEqual(
            len(encoded), TRACE_REPORT_HEADER.size + TRACE_SAMPLE_ABSOLUTE.size
        )
        self.assertEqual(decode_trace_report(encoded), report)

    def test_multiple_trace_samples_round_trip_and_match_golden_bytes(self):
        report = golden_report()
        encoded = encode_trace_report(report)
        self.assertEqual(encoded.hex(), GOLDEN_TRACE_REPORT_HEX)
        self.assertEqual(decode_trace_report(encoded), report)

    def test_fifteen_samples_fit_payload_limit(self):
        samples = tuple(
            make_sample(
                uptime_ms=1000 + index * 100,
                x_cm=-100 + index,
                y_cm=200 - index * 2,
                z_cm=-300 + index * 3,
                heading_cdeg=index * 100,
                quality=index % 5,
                flags=index % 4,
            )
            for index in range(15)
        )
        encoded = encode_trace_report(make_report(samples))
        self.assertEqual(len(encoded), 212)
        self.assertLessEqual(len(encoded), MAX_PAYLOAD_LEN)
        self.assertEqual(decode_trace_report(encoded).samples, samples)

    def test_sixteen_samples_are_rejected(self):
        samples = tuple(
            make_sample(uptime_ms=1000 + index, x_cm=index)
            for index in range(16)
        )
        with self.assertRaises(ProtocolError):
            encode_trace_report(make_report(samples))

        raw = TRACE_REPORT_HEADER.pack(1, 2, 3, 1, 1, 16, 16, 0)
        raw += TRACE_SAMPLE_ABSOLUTE.pack(1, 0, 0, 0, 0, 0, 0)
        raw += TRACE_SAMPLE_DELTA.pack(1, 0, 0, 0, 0, 0, 0) * 15
        with self.assertRaises(ProtocolError):
            decode_trace_report(raw)

    def test_negative_coordinates_round_trip(self):
        sample = make_sample(x_cm=-1, y_cm=-0x80000000, z_cm=-123456)
        report = make_report((sample,))
        self.assertEqual(decode_trace_report(encode_trace_report(report)), report)

    def test_maximum_int16_deltas_are_accepted(self):
        first = make_sample(uptime_ms=1, x_cm=0, y_cm=0, z_cm=0)
        second = make_sample(
            uptime_ms=2,
            x_cm=0x7FFF,
            y_cm=-0x8000,
            z_cm=0x7FFF,
            heading_cdeg=35999,
        )
        report = make_report((first, second))
        self.assertEqual(decode_trace_report(encode_trace_report(report)), report)

    def test_out_of_range_int16_delta_is_rejected(self):
        first = make_sample(uptime_ms=1, x_cm=0, y_cm=0, z_cm=0)
        for field in ("x_cm", "y_cm", "z_cm"):
            values = {"uptime_ms": 2, "x_cm": 0, "y_cm": 0, "z_cm": 0}
            values[field] = 0x8000
            with self.subTest(field=field), self.assertRaises(ProtocolError):
                encode_trace_report(make_report((first, make_sample(**values))))

    def test_dt_ms_65535_is_accepted(self):
        first = make_sample(uptime_ms=100)
        second = make_sample(uptime_ms=100 + 0xFFFF, x_cm=11)
        report = make_report((first, second))
        self.assertEqual(decode_trace_report(encode_trace_report(report)), report)

    def test_uptime_overflow_is_rejected_while_decoding(self):
        raw = TRACE_REPORT_HEADER.pack(1, 2, 3, 1, 1, 2, 2, 0)
        raw += TRACE_SAMPLE_ABSOLUTE.pack(0xFFFFFFFF, 0, 0, 0, 0, 0, 0)
        raw += TRACE_SAMPLE_DELTA.pack(1, 0, 0, 0, 0, 0, 0)
        with self.assertRaises(ProtocolError):
            decode_trace_report(raw)

    def test_heading_above_35999_is_rejected(self):
        with self.assertRaises(ProtocolError):
            encode_trace_report(make_report((make_sample(heading_cdeg=36000),)))

        raw = TRACE_REPORT_HEADER.pack(1, 2, 3, 1, 1, 1, 1, 0)
        raw += TRACE_SAMPLE_ABSOLUTE.pack(1, 0, 0, 0, 36000, 0, 0)
        with self.assertRaises(ProtocolError):
            decode_trace_report(raw)

    def test_quality_above_four_is_rejected(self):
        with self.assertRaises(ProtocolError):
            encode_trace_report(make_report((make_sample(quality=5),)))

        raw = TRACE_REPORT_HEADER.pack(1, 2, 3, 1, 1, 1, 1, 0)
        raw += TRACE_SAMPLE_ABSOLUTE.pack(1, 0, 0, 0, 0, 5, 0)
        with self.assertRaises(ProtocolError):
            decode_trace_report(raw)

    def test_truncated_trace_report_is_rejected(self):
        encoded = bytes.fromhex(GOLDEN_TRACE_REPORT_HEX)
        for truncated in (encoded[: TRACE_REPORT_HEADER.size - 1], encoded[:-1]):
            with self.subTest(length=len(truncated)), self.assertRaises(ProtocolError):
                decode_trace_report(truncated)

    def test_trace_report_with_extra_bytes_is_rejected(self):
        encoded = bytes.fromhex(GOLDEN_TRACE_REPORT_HEX)
        with self.assertRaises(ProtocolError):
            decode_trace_report(encoded + b"\x00")

        empty = encode_trace_report(make_report())
        with self.assertRaises(ProtocolError):
            decode_trace_report(empty + b"\x00")

    def test_unknown_trace_report_flags_are_rejected(self):
        with self.assertRaises(ProtocolError):
            encode_trace_report(make_report(report_flags=0x08))

        raw = TRACE_REPORT_HEADER.pack(1, 2, 3, 0, 0, 0, 0, 0x08)
        with self.assertRaises(ProtocolError):
            decode_trace_report(raw)

    def test_zero_delta_time_is_rejected_while_decoding(self):
        raw = TRACE_REPORT_HEADER.pack(1, 2, 3, 1, 1, 2, 2, 0)
        raw += TRACE_SAMPLE_ABSOLUTE.pack(1, 0, 0, 0, 0, 0, 0)
        raw += TRACE_SAMPLE_DELTA.pack(0, 0, 0, 0, 0, 0, 0)
        with self.assertRaises(ProtocolError):
            decode_trace_report(raw)

    def test_recovered_int32_overflow_is_rejected_while_decoding(self):
        raw = TRACE_REPORT_HEADER.pack(1, 2, 3, 1, 1, 2, 2, 0)
        raw += TRACE_SAMPLE_ABSOLUTE.pack(1, 0x7FFFFFFF, 0, 0, 0, 0, 0)
        raw += TRACE_SAMPLE_DELTA.pack(1, 1, 0, 0, 0, 0, 0)
        with self.assertRaises(ProtocolError):
            decode_trace_report(raw)

    def test_nonempty_report_requires_valid_sequence_bounds(self):
        with self.assertRaises(ProtocolError):
            encode_trace_report(make_report((make_sample(),), first_sample_seq=0))
        with self.assertRaises(ProtocolError):
            encode_trace_report(
                make_report((make_sample(),), first_sample_seq=2, latest_available_seq=1)
            )


if __name__ == "__main__":
    unittest.main()
