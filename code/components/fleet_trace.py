"""Bounded local car pose history and hardware-free FleetBus trace sampling."""

from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Deque, Optional, Tuple

from .fleet_models import (
    NodeFlags,
    TraceReportFlags,
    TraceReportPayload,
    TraceRequestPayload,
    TraceSample,
    TraceSampleFlags,
)
from .fleet_protocol import (
    TRACE_MAX_SAMPLES,
    encode_trace_request,
    new_session,
    validate_trace_sample,
)


@dataclass(frozen=True)
class StoredTraceSample:
    sample_seq: int
    uptime_ms: int
    x_cm: int
    y_cm: int
    z_cm: int
    heading_cdeg: int
    quality: int
    flags: int


@dataclass(frozen=True)
class TraceSamplingOptions:
    enabled: bool = False
    sample_interval_s: float = 0.10
    buffer_capacity: int = 600
    min_distance_cm: float = 1.0
    stationary_keepalive_s: float = 1.0

    def __post_init__(self) -> None:
        if not math.isfinite(float(self.sample_interval_s)) or self.sample_interval_s <= 0:
            raise ValueError("sample_interval_s must be positive")
        if not isinstance(self.buffer_capacity, int) or self.buffer_capacity <= 0:
            raise ValueError("buffer_capacity must be a positive integer")
        if not math.isfinite(float(self.min_distance_cm)) or self.min_distance_cm < 0:
            raise ValueError("min_distance_cm must not be negative")
        if (
            not math.isfinite(float(self.stationary_keepalive_s))
            or self.stationary_keepalive_s <= 0
        ):
            raise ValueError("stationary_keepalive_s must be positive")


def new_nonzero_session() -> int:
    while True:
        value = new_session()
        if value != 0:
            return value


def car_state_to_trace_sample(state: object) -> TraceSample:
    flags = TraceSampleFlags.NONE
    node_flags = int(state.node_flags)
    if node_flags & int(NodeFlags.POSE_VALID):
        flags |= TraceSampleFlags.POSE_VALID
    if node_flags & int(NodeFlags.LOCALIZATION_DEGRADED):
        flags |= TraceSampleFlags.LOCALIZATION_DEGRADED
    return TraceSample(
        uptime_ms=int(state.uptime_ms),
        x_cm=int(state.x_cm),
        y_cm=int(state.y_cm),
        z_cm=0,
        heading_cdeg=int(state.heading_cdeg),
        quality=int(state.pose_quality),
        flags=int(flags),
    )


class PoseTraceBuffer:
    def __init__(
        self,
        options: TraceSamplingOptions = TraceSamplingOptions(),
        session_factory: Callable[[], int] = new_nonzero_session,
    ) -> None:
        self._options = options
        self._session_factory = session_factory
        self._samples = deque(maxlen=options.buffer_capacity)  # type: Deque[StoredTraceSample]
        self._trace_session = self._next_session(None)
        self._next_sample_seq = 1
        self._lock = threading.Lock()
        self.overwritten_samples = 0
        self.recorded_samples = 0
        self.skipped_stationary_samples = 0
        self.stream_resets = 0

    @property
    def trace_session(self) -> int:
        with self._lock:
            return self._trace_session

    @property
    def sample_count(self) -> int:
        with self._lock:
            return len(self._samples)

    def latest_cursor(self) -> Tuple[int, int]:
        """Return one lock-consistent trace session and latest sample sequence."""
        with self._lock:
            latest_sample_seq = self._samples[-1].sample_seq if self._samples else 0
            return self._trace_session, latest_sample_seq

    def _next_session(self, previous: Optional[int]) -> int:
        while True:
            value = self._session_factory()
            if isinstance(value, int) and 0 < value <= 0xFFFFFFFF and value != previous:
                return value

    def _reset_stream_locked(self) -> None:
        previous = self._trace_session
        self._samples.clear()
        self._trace_session = self._next_session(previous)
        self._next_sample_seq = 1
        self.stream_resets += 1

    def record(self, sample: TraceSample) -> bool:
        validate_trace_sample(sample)
        with self._lock:
            previous = self._samples[-1] if self._samples else None
            if previous is not None and sample.uptime_ms <= previous.uptime_ms:
                self._reset_stream_locked()
                previous = None

            if previous is not None:
                dx_cm = sample.x_cm - previous.x_cm
                dy_cm = sample.y_cm - previous.y_cm
                dz_cm = sample.z_cm - previous.z_cm
                distance_cm = math.sqrt(
                    dx_cm * dx_cm + dy_cm * dy_cm + dz_cm * dz_cm
                )
                elapsed_ms = sample.uptime_ms - previous.uptime_ms
                if (
                    distance_cm < self._options.min_distance_cm
                    and sample.flags == previous.flags
                    and sample.quality == previous.quality
                    and elapsed_ms < self._options.stationary_keepalive_s * 1000.0
                ):
                    self.skipped_stationary_samples += 1
                    return False

            if self._next_sample_seq > 0xFFFFFFFF:
                self._reset_stream_locked()
            if len(self._samples) == self._samples.maxlen:
                self.overwritten_samples += 1
            self._samples.append(
                StoredTraceSample(
                    self._next_sample_seq,
                    sample.uptime_ms,
                    sample.x_cm,
                    sample.y_cm,
                    sample.z_cm,
                    sample.heading_cdeg,
                    sample.quality,
                    sample.flags,
                )
            )
            self._next_sample_seq += 1
            self.recorded_samples += 1
            return True

    @staticmethod
    def _as_trace_sample(sample: StoredTraceSample) -> TraceSample:
        return TraceSample(
            sample.uptime_ms,
            sample.x_cm,
            sample.y_cm,
            sample.z_cm,
            sample.heading_cdeg,
            sample.quality,
            sample.flags,
        )

    @staticmethod
    def _delta_encodable(previous: TraceSample, current: TraceSample) -> bool:
        return (
            1 <= current.uptime_ms - previous.uptime_ms <= 0xFFFF
            and -0x8000 <= current.x_cm - previous.x_cm <= 0x7FFF
            and -0x8000 <= current.y_cm - previous.y_cm <= 0x7FFF
            and -0x8000 <= current.z_cm - previous.z_cm <= 0x7FFF
        )

    def build_report(
        self,
        request_session: int,
        request_seq: int,
        request: TraceRequestPayload,
    ) -> TraceReportPayload:
        encode_trace_request(request)
        with self._lock:
            trace_session = self._trace_session
            snapshot = tuple(self._samples)

        if not snapshot:
            return TraceReportPayload(
                request_session,
                request_seq,
                trace_session,
                0,
                0,
                0,
                int(TraceReportFlags.NONE),
                (),
            )

        oldest_available_seq = snapshot[0].sample_seq
        latest_available_seq = snapshot[-1].sample_seq
        report_flags = TraceReportFlags.NONE
        start_index = len(snapshot)
        if request.known_trace_session != trace_session:
            start_index = 0
            report_flags |= TraceReportFlags.CURSOR_RESET
        elif request.after_sample_seq > latest_available_seq:
            start_index = 0
            report_flags |= TraceReportFlags.CURSOR_RESET
        elif request.after_sample_seq == latest_available_seq:
            start_index = len(snapshot)
        elif request.after_sample_seq + 1 < oldest_available_seq:
            start_index = 0
            report_flags |= TraceReportFlags.BUFFER_OVERRUN
        else:
            start_index = request.after_sample_seq + 1 - oldest_available_seq

        selected = []
        for stored in snapshot[
            start_index : start_index + min(request.max_samples, TRACE_MAX_SAMPLES)
        ]:
            sample = self._as_trace_sample(stored)
            if selected and not self._delta_encodable(selected[-1], sample):
                break
            selected.append(sample)

        first_sample_seq = snapshot[start_index].sample_seq if selected else 0
        if selected and first_sample_seq + len(selected) - 1 < latest_available_seq:
            report_flags |= TraceReportFlags.MORE_PENDING
        return TraceReportPayload(
            request_session,
            request_seq,
            trace_session,
            oldest_available_seq,
            first_sample_seq,
            latest_available_seq,
            int(report_flags),
            tuple(selected),
        )


class PoseTraceSampler:
    def __init__(
        self,
        *,
        state_provider: Callable[[], object],
        trace_buffer: PoseTraceBuffer,
        options: TraceSamplingOptions,
        state_adapter: Callable[[object], TraceSample],
        monotonic: Callable[[], float] = time.monotonic,
        wait: Optional[Callable[[threading.Event, float], object]] = None,
    ) -> None:
        self._state_provider = state_provider
        self._trace_buffer = trace_buffer
        self._options = options
        self._state_adapter = state_adapter
        self._monotonic = monotonic
        self._wait = wait or (lambda stop_event, timeout: stop_event.wait(timeout))
        self._stop = threading.Event()
        self._thread = None  # type: Optional[threading.Thread]
        self._lifecycle_lock = threading.Lock()
        self._error_lock = threading.Lock()
        self.sample_errors = 0
        self.last_error = None  # type: Optional[str]

    @property
    def running(self) -> bool:
        with self._lifecycle_lock:
            return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if not self._options.enabled:
            return
        with self._lifecycle_lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._run,
                name="fleet-car-pose-trace-sampler",
                daemon=True,
            )
            self._thread.start()

    def close(self) -> None:
        self._stop.set()
        with self._lifecycle_lock:
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lifecycle_lock:
            if self._thread is thread and (thread is None or not thread.is_alive()):
                self._thread = None

    def _sample_once(self) -> None:
        try:
            state = self._state_provider()
            sample = self._state_adapter(state)
            self._trace_buffer.record(sample)
        except Exception as exc:
            with self._error_lock:
                self.sample_errors += 1
                self.last_error = "{}: {}".format(type(exc).__name__, exc)

    def _run(self) -> None:
        interval = self._options.sample_interval_s
        next_due = self._monotonic()
        while not self._stop.is_set():
            now = self._monotonic()
            if now < next_due:
                self._wait(self._stop, next_due - now)
                continue
            self._sample_once()
            next_due += interval
            now = self._monotonic()
            if next_due < now - interval:
                next_due = now + interval
