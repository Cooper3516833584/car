"""Hardware-free tests for local trace buffering and sampling."""

from pathlib import Path
from types import SimpleNamespace
import math
import sys
import threading
import time
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.fleet_models import (  # noqa: E402
    NodeFlags,
    TraceReportFlags,
    TraceRequestPayload,
    TraceSample,
    TraceSampleFlags,
)
from components.fleet_trace import (  # noqa: E402
    PoseTraceBuffer,
    PoseTraceSampler,
    TraceSamplingOptions,
    car_state_to_trace_sample,
)
from components.fleet_protocol import ProtocolError  # noqa: E402


def make_sample(
    uptime_ms=1000,
    x_cm=0,
    y_cm=0,
    z_cm=0,
    heading_cdeg=0,
    quality=4,
    flags=int(TraceSampleFlags.POSE_VALID),
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


def make_session_factory(*values):
    sessions = iter(values)
    return lambda: next(sessions)


def make_buffer(
    *,
    capacity=600,
    min_distance_cm=0.0,
    keepalive_s=1.0,
    sessions=(0x11111111, 0x22222222, 0x33333333),
):
    return PoseTraceBuffer(
        TraceSamplingOptions(
            buffer_capacity=capacity,
            min_distance_cm=min_distance_cm,
            stationary_keepalive_s=keepalive_s,
        ),
        session_factory=make_session_factory(*sessions),
    )


def record_samples(buffer, count, *, start_ms=1000, step_ms=100):
    for index in range(count):
        accepted = buffer.record(
            make_sample(
                uptime_ms=start_ms + index * step_ms,
                x_cm=index * 10,
                y_cm=-index * 2,
                z_cm=index * 3,
                heading_cdeg=index * 100,
            )
        )
        if not accepted:
            raise AssertionError("test fixture sample was unexpectedly skipped")


def build_report(buffer, *, after=0, known_session=None, max_samples=15):
    if known_session is None:
        known_session = buffer.trace_session
    return buffer.build_report(
        request_session=0xAABBCCDD,
        request_seq=0x1234,
        request=TraceRequestPayload(
            known_trace_session=known_session,
            after_sample_seq=after,
            max_samples=max_samples,
            flags=0,
        ),
    )


class PoseTraceBufferTests(unittest.TestCase):
    def test_first_sample_sequence_is_one(self):
        buffer = make_buffer()
        sample = make_sample()
        self.assertTrue(buffer.record(sample))
        report = build_report(buffer, known_session=0)
        self.assertEqual(report.first_sample_seq, 1)
        self.assertEqual(report.oldest_available_seq, 1)
        self.assertEqual(report.latest_available_seq, 1)
        self.assertEqual(report.samples, (sample,))

    def test_subsequent_sample_sequences_are_contiguous(self):
        buffer = make_buffer()
        record_samples(buffer, 4)
        report = build_report(buffer, known_session=0)
        sequences = tuple(
            report.first_sample_seq + index for index in range(len(report.samples))
        )
        self.assertEqual(sequences, (1, 2, 3, 4))
        self.assertEqual(report.latest_available_seq, 4)

    def test_short_stationary_duplicate_is_skipped(self):
        buffer = make_buffer(min_distance_cm=1.0, keepalive_s=1.0)
        self.assertTrue(buffer.record(make_sample(uptime_ms=1000, heading_cdeg=0)))
        self.assertFalse(buffer.record(make_sample(uptime_ms=1500, heading_cdeg=9000)))
        self.assertEqual(buffer.sample_count, 1)
        self.assertEqual(buffer.skipped_stationary_samples, 1)

    def test_stationary_sample_at_keepalive_is_retained(self):
        buffer = make_buffer(min_distance_cm=1.0, keepalive_s=1.0)
        self.assertTrue(buffer.record(make_sample(uptime_ms=1000)))
        self.assertTrue(buffer.record(make_sample(uptime_ms=2000)))
        report = build_report(buffer, known_session=0)
        self.assertEqual(len(report.samples), 2)
        self.assertEqual(report.latest_available_seq, 2)

    def test_position_change_at_distance_threshold_is_retained(self):
        buffer = make_buffer(min_distance_cm=5.0)
        self.assertTrue(buffer.record(make_sample(uptime_ms=1000)))
        self.assertTrue(buffer.record(make_sample(uptime_ms=1100, x_cm=3, y_cm=4)))
        self.assertEqual(buffer.sample_count, 2)

    def test_flag_change_retains_stationary_sample(self):
        buffer = make_buffer(min_distance_cm=1.0)
        self.assertTrue(buffer.record(make_sample(uptime_ms=1000, flags=0)))
        self.assertTrue(
            buffer.record(
                make_sample(
                    uptime_ms=1100,
                    flags=int(TraceSampleFlags.POSE_VALID),
                )
            )
        )
        self.assertEqual(buffer.sample_count, 2)

    def test_quality_change_retains_stationary_sample(self):
        buffer = make_buffer(min_distance_cm=1.0)
        self.assertTrue(buffer.record(make_sample(uptime_ms=1000, quality=4)))
        self.assertTrue(buffer.record(make_sample(uptime_ms=1100, quality=3)))
        self.assertEqual(buffer.sample_count, 2)

    def test_uptime_rollback_changes_trace_session(self):
        buffer = make_buffer(sessions=(11, 22))
        self.assertTrue(buffer.record(make_sample(uptime_ms=1000)))
        old_session = buffer.trace_session
        self.assertTrue(buffer.record(make_sample(uptime_ms=999)))
        self.assertEqual(old_session, 11)
        self.assertEqual(buffer.trace_session, 22)
        self.assertEqual(buffer.stream_resets, 1)

    def test_uptime_rollback_restarts_sequence_at_one(self):
        buffer = make_buffer(sessions=(11, 22))
        buffer.record(make_sample(uptime_ms=1000))
        replacement = make_sample(uptime_ms=999, x_cm=50)
        buffer.record(replacement)
        report = build_report(buffer, known_session=0)
        self.assertEqual(report.first_sample_seq, 1)
        self.assertEqual(report.latest_available_seq, 1)
        self.assertEqual(report.samples, (replacement,))

    def test_ring_buffer_overwrites_oldest_samples(self):
        buffer = make_buffer(capacity=3)
        record_samples(buffer, 5)
        report = build_report(buffer, known_session=0)
        self.assertEqual(report.oldest_available_seq, 3)
        self.assertEqual(report.first_sample_seq, 3)
        self.assertEqual(report.latest_available_seq, 5)
        self.assertEqual(tuple(sample.x_cm for sample in report.samples), (20, 30, 40))

    def test_ring_buffer_overwrite_counter_is_exact(self):
        buffer = make_buffer(capacity=3)
        record_samples(buffer, 5)
        self.assertEqual(buffer.overwritten_samples, 2)
        self.assertEqual(buffer.recorded_samples, 5)
        self.assertEqual(buffer.sample_count, 3)

    def test_unknown_trace_session_returns_from_oldest_sample(self):
        buffer = make_buffer(capacity=3)
        record_samples(buffer, 5)
        report = build_report(buffer, known_session=0, after=999)
        self.assertEqual(report.first_sample_seq, 3)
        self.assertEqual(len(report.samples), 3)
        self.assertTrue(report.report_flags & int(TraceReportFlags.CURSOR_RESET))
        self.assertFalse(report.report_flags & int(TraceReportFlags.BUFFER_OVERRUN))

    def test_normal_cursor_returns_next_sample(self):
        buffer = make_buffer()
        record_samples(buffer, 5)
        report = build_report(buffer, after=2)
        self.assertEqual(report.first_sample_seq, 3)
        self.assertEqual(len(report.samples), 3)
        self.assertEqual(report.samples[0].x_cm, 20)

    def test_cursor_at_latest_sample_returns_empty_batch(self):
        buffer = make_buffer()
        record_samples(buffer, 3)
        report = build_report(buffer, after=3)
        self.assertEqual(report.first_sample_seq, 0)
        self.assertEqual(report.samples, ())
        self.assertEqual(report.oldest_available_seq, 1)
        self.assertEqual(report.latest_available_seq, 3)
        self.assertEqual(report.report_flags, int(TraceReportFlags.NONE))

    def test_cursor_behind_oldest_sets_buffer_overrun(self):
        buffer = make_buffer(capacity=3)
        record_samples(buffer, 5)
        report = build_report(buffer, after=1)
        self.assertEqual(report.first_sample_seq, 3)
        self.assertTrue(report.report_flags & int(TraceReportFlags.BUFFER_OVERRUN))
        self.assertFalse(report.report_flags & int(TraceReportFlags.CURSOR_RESET))

    def test_future_cursor_resets_to_oldest_sample(self):
        buffer = make_buffer(capacity=3)
        record_samples(buffer, 5)
        report = build_report(buffer, after=99)
        self.assertEqual(report.first_sample_seq, 3)
        self.assertTrue(report.report_flags & int(TraceReportFlags.CURSOR_RESET))
        self.assertFalse(report.report_flags & int(TraceReportFlags.BUFFER_OVERRUN))

    def test_batch_size_respects_requested_limit(self):
        buffer = make_buffer()
        record_samples(buffer, 5)
        report = build_report(buffer, after=0, max_samples=2)
        self.assertEqual(len(report.samples), 2)
        self.assertEqual(report.first_sample_seq, 1)
        self.assertTrue(report.report_flags & int(TraceReportFlags.MORE_PENDING))

    def test_batch_never_exceeds_protocol_limit(self):
        buffer = make_buffer()
        record_samples(buffer, 20)
        report = build_report(buffer, after=0, max_samples=15)
        self.assertEqual(len(report.samples), 15)
        self.assertLessEqual(len(report.samples), 15)

    def test_unencodable_delta_ends_batch_before_sample(self):
        buffer = make_buffer()
        first = make_sample(uptime_ms=1000, x_cm=0)
        second = make_sample(uptime_ms=1100, x_cm=1)
        third = make_sample(uptime_ms=1200, x_cm=40000)
        for sample in (first, second, third):
            self.assertTrue(buffer.record(sample))
        report = build_report(buffer, known_session=0)
        self.assertEqual(report.first_sample_seq, 1)
        self.assertEqual(report.samples, (first, second))
        self.assertTrue(report.report_flags & int(TraceReportFlags.MORE_PENDING))

    def test_unencodable_delta_sample_is_next_batch_absolute_first(self):
        buffer = make_buffer()
        samples = (
            make_sample(uptime_ms=1000, x_cm=0),
            make_sample(uptime_ms=1100, x_cm=1),
            make_sample(uptime_ms=1200, x_cm=40000),
        )
        for sample in samples:
            buffer.record(sample)
        first_report = build_report(buffer, known_session=0)
        second_report = build_report(buffer, after=2)
        self.assertEqual(len(first_report.samples), 2)
        self.assertEqual(second_report.first_sample_seq, 3)
        self.assertEqual(second_report.samples, (samples[2],))

    def test_more_pending_tracks_remaining_samples(self):
        buffer = make_buffer()
        record_samples(buffer, 4)
        first = build_report(buffer, after=0, max_samples=2)
        final = build_report(buffer, after=2, max_samples=2)
        empty = build_report(buffer, after=4, max_samples=2)
        self.assertTrue(first.report_flags & int(TraceReportFlags.MORE_PENDING))
        self.assertFalse(final.report_flags & int(TraceReportFlags.MORE_PENDING))
        self.assertFalse(empty.report_flags & int(TraceReportFlags.MORE_PENDING))

    def test_build_report_does_not_delete_samples(self):
        buffer = make_buffer()
        record_samples(buffer, 3)
        before = (buffer.sample_count, buffer.recorded_samples)
        build_report(buffer, known_session=0)
        self.assertEqual((buffer.sample_count, buffer.recorded_samples), before)

    def test_repeated_request_with_same_cursor_is_idempotent(self):
        buffer = make_buffer()
        record_samples(buffer, 5)
        first = build_report(buffer, after=1, max_samples=3)
        repeated = build_report(buffer, after=1, max_samples=3)
        self.assertEqual(repeated, first)

    def test_empty_buffer_report_has_zero_cursors(self):
        buffer = make_buffer()
        report = build_report(buffer, known_session=0)
        self.assertEqual(report.oldest_available_seq, 0)
        self.assertEqual(report.first_sample_seq, 0)
        self.assertEqual(report.latest_available_seq, 0)
        self.assertEqual(report.samples, ())

    def test_sequence_overflow_resets_stream_before_next_sample(self):
        buffer = make_buffer(sessions=(11, 22))
        buffer.record(make_sample(uptime_ms=1000))
        with buffer._lock:
            buffer._next_sample_seq = 0x100000000
        replacement = make_sample(uptime_ms=1100, x_cm=1)
        buffer.record(replacement)
        report = build_report(buffer, known_session=0)
        self.assertEqual(buffer.trace_session, 22)
        self.assertEqual(buffer.stream_resets, 1)
        self.assertEqual(report.first_sample_seq, 1)
        self.assertEqual(report.samples, (replacement,))

    def test_session_factory_retries_invalid_or_repeated_values(self):
        buffer = make_buffer(sessions=(0, -1, 0x100000000, 7, 7, 8))
        self.assertEqual(buffer.trace_session, 7)
        buffer.record(make_sample(uptime_ms=1000))
        buffer.record(make_sample(uptime_ms=999))
        self.assertEqual(buffer.trace_session, 8)

    def test_sampling_options_validate_all_numeric_bounds(self):
        invalid_options = (
            {"sample_interval_s": 0},
            {"sample_interval_s": math.nan},
            {"buffer_capacity": 0},
            {"buffer_capacity": 1.5},
            {"min_distance_cm": -1},
            {"min_distance_cm": math.inf},
            {"stationary_keepalive_s": 0},
            {"stationary_keepalive_s": math.nan},
        )
        for values in invalid_options:
            with self.subTest(values=values), self.assertRaises(ValueError):
                TraceSamplingOptions(**values)

    def test_invalid_samples_are_rejected_without_polluting_buffer(self):
        invalid_samples = (
            make_sample(heading_cdeg=36000),
            make_sample(quality=5),
            make_sample(x_cm=0x80000000),
            make_sample(uptime_ms=-1),
            make_sample(flags=0x100),
        )
        for sample in invalid_samples:
            buffer = make_buffer()
            with self.subTest(sample=sample), self.assertRaises(ProtocolError):
                buffer.record(sample)
            self.assertEqual(buffer.sample_count, 0)
            self.assertEqual(buffer.recorded_samples, 0)
            self.assertEqual(buffer.stream_resets, 0)


class _FakeClock:
    def __init__(self):
        self.now = 0.0
        self.waits = []

    def monotonic(self):
        return self.now

    def wait(self, stop_event, timeout):
        if timeout < 0:
            raise AssertionError("sampler requested a negative wait")
        self.waits.append(timeout)
        self.now += timeout
        return stop_event.is_set()


class _RecordingBuffer:
    def __init__(self):
        self.samples = []

    def record(self, sample):
        self.samples.append(sample)
        return True


class PoseTraceSamplerTests(unittest.TestCase):
    def _run_with_fake_clock(self, *, sample_count, processing_s):
        clock = _FakeClock()
        buffer = _RecordingBuffer()
        sample_times = []
        sampler = None

        def state_provider():
            sample_times.append(clock.now)
            clock.now += processing_s
            if len(sample_times) == sample_count:
                sampler._stop.set()
            return len(sample_times)

        sampler = PoseTraceSampler(
            state_provider=state_provider,
            trace_buffer=buffer,
            options=TraceSamplingOptions(enabled=True, sample_interval_s=0.1),
            state_adapter=lambda value: make_sample(
                uptime_ms=value * 100,
                x_cm=value,
            ),
            monotonic=clock.monotonic,
            wait=clock.wait,
        )
        sampler._run()
        return clock, buffer, sample_times

    def test_sampler_runs_on_ten_hertz_schedule(self):
        _, buffer, sample_times = self._run_with_fake_clock(
            sample_count=5, processing_s=0.0
        )
        self.assertEqual(len(buffer.samples), 5)
        for actual, expected in zip(sample_times, (0.0, 0.1, 0.2, 0.3, 0.4)):
            self.assertAlmostEqual(actual, expected)

    def test_sampler_processing_time_does_not_accumulate_drift(self):
        clock, _, sample_times = self._run_with_fake_clock(
            sample_count=5, processing_s=0.03
        )
        for actual, expected in zip(sample_times, (0.0, 0.1, 0.2, 0.3, 0.4)):
            self.assertAlmostEqual(actual, expected)
        for wait in clock.waits:
            self.assertAlmostEqual(wait, 0.07)

    def test_sampler_skips_unbounded_catch_up_after_long_processing(self):
        _, _, sample_times = self._run_with_fake_clock(
            sample_count=3, processing_s=0.25
        )
        self.assertAlmostEqual(sample_times[0], 0.0)
        self.assertAlmostEqual(sample_times[1], 0.35)
        self.assertAlmostEqual(sample_times[2], 0.70)

    def test_state_provider_error_does_not_stop_sampling(self):
        clock = _FakeClock()
        buffer = _RecordingBuffer()
        attempts = []
        sampler = None

        def state_provider():
            attempts.append(clock.now)
            if len(attempts) == 1:
                raise RuntimeError("boom")
            if len(attempts) == 3:
                sampler._stop.set()
            return len(attempts)

        sampler = PoseTraceSampler(
            state_provider=state_provider,
            trace_buffer=buffer,
            options=TraceSamplingOptions(enabled=True, sample_interval_s=0.1),
            state_adapter=lambda value: make_sample(value * 100, x_cm=value),
            monotonic=clock.monotonic,
            wait=clock.wait,
        )
        sampler._run()
        self.assertEqual(len(attempts), 3)
        self.assertEqual(len(buffer.samples), 2)
        self.assertEqual(sampler.sample_errors, 1)
        self.assertEqual(sampler.last_error, "RuntimeError: boom")

    def test_state_adapter_error_does_not_stop_sampling(self):
        clock = _FakeClock()
        buffer = _RecordingBuffer()
        attempts = []
        sampler = None

        def state_provider():
            attempts.append(clock.now)
            if len(attempts) == 2:
                sampler._stop.set()
            return len(attempts)

        def state_adapter(value):
            if value == 1:
                raise ValueError("bad state")
            return make_sample(value * 100, x_cm=value)

        sampler = PoseTraceSampler(
            state_provider=state_provider,
            trace_buffer=buffer,
            options=TraceSamplingOptions(enabled=True, sample_interval_s=0.1),
            state_adapter=state_adapter,
            monotonic=clock.monotonic,
            wait=clock.wait,
        )
        sampler._run()
        self.assertEqual(len(buffer.samples), 1)
        self.assertEqual(sampler.sample_errors, 1)
        self.assertEqual(sampler.last_error, "ValueError: bad state")

    def test_invalid_sample_error_does_not_block_next_valid_sample(self):
        clock = _FakeClock()
        buffer = make_buffer()
        attempts = []
        sampler = None

        def state_provider():
            attempts.append(clock.now)
            if len(attempts) == 2:
                sampler._stop.set()
            return len(attempts)

        def state_adapter(value):
            if value == 1:
                return make_sample(uptime_ms=100, heading_cdeg=36000)
            return make_sample(uptime_ms=200, x_cm=2)

        sampler = PoseTraceSampler(
            state_provider=state_provider,
            trace_buffer=buffer,
            options=TraceSamplingOptions(enabled=True, sample_interval_s=0.1),
            state_adapter=state_adapter,
            monotonic=clock.monotonic,
            wait=clock.wait,
        )
        sampler._run()
        report = build_report(buffer, known_session=0)
        self.assertEqual(sampler.sample_errors, 1)
        self.assertIn("heading_cdeg", sampler.last_error)
        self.assertEqual(report.first_sample_seq, 1)
        self.assertEqual(report.samples, (make_sample(uptime_ms=200, x_cm=2),))

    def test_repeated_start_does_not_create_multiple_threads(self):
        entered = threading.Event()
        release = threading.Event()

        def state_provider():
            entered.set()
            release.wait(0.5)
            return object()

        sampler = PoseTraceSampler(
            state_provider=state_provider,
            trace_buffer=_RecordingBuffer(),
            options=TraceSamplingOptions(enabled=True, sample_interval_s=10.0),
            state_adapter=lambda _: make_sample(),
        )
        sampler.start()
        try:
            self.assertTrue(entered.wait(0.5))
            first_thread = sampler._thread
            sampler.start()
            self.assertIs(sampler._thread, first_thread)
            self.assertTrue(sampler.running)
        finally:
            release.set()
            sampler.close()
        self.assertFalse(sampler.running)

    def test_close_wakes_waiting_sampler_promptly(self):
        sampled = threading.Event()

        def state_provider():
            sampled.set()
            return object()

        sampler = PoseTraceSampler(
            state_provider=state_provider,
            trace_buffer=_RecordingBuffer(),
            options=TraceSamplingOptions(enabled=True, sample_interval_s=60.0),
            state_adapter=lambda _: make_sample(),
        )
        sampler.start()
        try:
            self.assertTrue(sampled.wait(0.5))
            started = time.monotonic()
            sampler.close()
            elapsed = time.monotonic() - started
        finally:
            sampler.close()
        self.assertLess(elapsed, 0.5)
        self.assertFalse(sampler.running)

    def test_repeated_close_is_safe(self):
        sampler = PoseTraceSampler(
            state_provider=lambda: object(),
            trace_buffer=_RecordingBuffer(),
            options=TraceSamplingOptions(enabled=True),
            state_adapter=lambda _: make_sample(),
        )
        sampler.close()
        sampler.close()
        self.assertFalse(sampler.running)

    def test_disabled_sampler_does_not_start_thread(self):
        calls = []
        sampler = PoseTraceSampler(
            state_provider=lambda: calls.append(1),
            trace_buffer=_RecordingBuffer(),
            options=TraceSamplingOptions(enabled=False),
            state_adapter=lambda _: make_sample(),
        )
        sampler.start()
        sampler.close()
        self.assertEqual(calls, [])
        self.assertFalse(sampler.running)

    def test_car_adapter_maps_all_pose_fields(self):
        state = SimpleNamespace(
            uptime_ms=1234,
            x_cm=-10,
            y_cm=20,
            z_cm=-30,
            heading_cdeg=35999,
            pose_quality=3,
            node_flags=int(NodeFlags.READY),
        )
        self.assertEqual(
            car_state_to_trace_sample(state),
            TraceSample(1234, -10, 20, 0, 35999, 3, 0),
        )

    def test_car_adapter_maps_pose_valid_flag(self):
        state = SimpleNamespace(
            uptime_ms=1,
            x_cm=0,
            y_cm=0,
            z_cm=0,
            heading_cdeg=0,
            pose_quality=4,
            node_flags=int(NodeFlags.POSE_VALID | NodeFlags.READY),
        )
        sample = car_state_to_trace_sample(state)
        self.assertEqual(sample.flags, int(TraceSampleFlags.POSE_VALID))

    def test_car_adapter_maps_localization_degraded_flag(self):
        state = SimpleNamespace(
            uptime_ms=1,
            x_cm=0,
            y_cm=0,
            z_cm=0,
            heading_cdeg=0,
            pose_quality=1,
            node_flags=int(
                NodeFlags.POSE_VALID | NodeFlags.LOCALIZATION_DEGRADED
            ),
        )
        sample = car_state_to_trace_sample(state)
        expected = TraceSampleFlags.POSE_VALID | TraceSampleFlags.LOCALIZATION_DEGRADED
        self.assertEqual(sample.flags, int(expected))


if __name__ == "__main__":
    unittest.main()
