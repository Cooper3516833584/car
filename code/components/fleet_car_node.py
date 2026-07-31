"""FleetBus V1 car node with a parse-only RX callback and one reply worker."""

import queue
import threading
import time
from typing import Callable, Optional

from .fleet_models import (
    AckPayload,
    AckReason,
    AckStatus,
    CarFleetState,
    CommandId,
    CommandResult,
    Frame,
    MapReportPayload,
    MessageKind,
    NodeId,
    NodeTiming,
    PathReportPayload,
    ReportPayload,
    TraceRequestPayload,
)
from .fleet_protocol import (
    FrameParser,
    ProtocolError,
    RecentResponseCache,
    SequenceCounter,
    VERSION,
    decode_car_navigate,
    decode_command,
    decode_coordinate_frame,
    decode_disaster_rescue,
    decode_trace_request,
    encode_ack,
    encode_map_report,
    encode_path_report,
    encode_report,
    encode_trace_report,
    new_session,
    pack_frame,
)
from .fleet_trace import (
    PoseTraceBuffer,
    PoseTraceSampler,
    TraceSamplingOptions,
    car_state_to_trace_sample,
)


class FleetCarNode:
    def __init__(
        self,
        *,
        writer: Callable[[bytes], None],
        state_provider: Callable[[], CarFleetState],
        on_set_coordinate_frame: Callable,
        on_navigate: Callable,
        on_stop: Callable[[], CommandResult],
        on_start_mapping: Optional[Callable[[int], CommandResult]] = None,
        on_set_alarm: Optional[Callable[[bool], CommandResult]] = None,
        on_start_mission: Optional[Callable[[], CommandResult]] = None,
        timing: NodeTiming = NodeTiming(),
        wait: Optional[Callable[[float], bool]] = None,
        trace_options: TraceSamplingOptions = TraceSamplingOptions(),
    ) -> None:
        self._writer = writer
        self._state_provider = state_provider
        self._on_set_coordinate_frame = on_set_coordinate_frame
        self._on_navigate = on_navigate
        self._on_stop = on_stop
        self._on_start_mapping = on_start_mapping
        self._on_set_alarm = on_set_alarm
        self._on_start_mission = on_start_mission
        self._on_disaster_rescue = None
        self._timing = timing
        self._parser = FrameParser(local_node=NodeId.CAR)
        self._queue = queue.PriorityQueue(maxsize=timing.queue_size)
        self._urgent_queue = queue.Queue(maxsize=4)
        self._stop_event = threading.Event()
        self._wait = self._stop_event.wait if wait is None else wait
        self._thread = None  # type: Optional[threading.Thread]
        self._cache = RecentResponseCache(64)
        self._session = new_session()
        self._seq = SequenceCounter()
        self._order = 0
        self._ground_session = None  # type: Optional[int]
        self.dropped_polls = 0
        self.dropped_requests = 0
        self._active_command_seq = 0
        self._active_command_status = 0
        self._error_code = 0
        self._trace_buffer = PoseTraceBuffer(trace_options)
        self._trace_progress = threading.Condition()
        self._observed_trace_session = 0
        self._observed_after_sample_seq = 0
        self._trace_sampler = (
            PoseTraceSampler(
                state_provider=state_provider,
                trace_buffer=self._trace_buffer,
                options=trace_options,
                state_adapter=car_state_to_trace_sample,
            )
            if trace_options.enabled
            else None
        )  # type: Optional[PoseTraceSampler]

    def set_disaster_handler(self, callback: Callable) -> None:
        """Install the task-layer rescue callback before accepting commands."""
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("disaster handler must be installed before FleetCarNode.start")
        self._on_disaster_rescue = callback

    @property
    def active_command_seq(self) -> int:
        """Return the request sequence currently represented by status reports."""
        return self._active_command_seq

    @property
    def trace_buffer(self) -> PoseTraceBuffer:
        return self._trace_buffer

    @property
    def trace_sampler(self) -> Optional[PoseTraceSampler]:
        return self._trace_sampler

    def set_active_command_result(
        self, result: CommandResult, request_seq: Optional[int] = None
    ) -> None:
        """Publish asynchronous task completion in subsequent state reports."""
        if request_seq is not None and self._active_command_seq != request_seq:
            return
        self._active_command_status = int(result.status)
        self._error_code = (
            int(result.reason)
            if result.status in (AckStatus.REJECTED, AckStatus.FAILED)
            else 0
        )

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        if self._trace_sampler is not None:
            self._trace_sampler.start()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, name="fleetbus-car-node", daemon=True
        )
        self._thread.start()

    def feed_frame(self, frame_bytes: bytes) -> None:
        for frame in self._parser.feed(frame_bytes):
            if frame.src != NodeId.GROUND or frame.dst != NodeId.CAR:
                continue
            self._order += 1
            priority = self._priority(frame)
            if priority == 0:
                try:
                    self._urgent_queue.put_nowait(frame)
                except queue.Full:
                    self.dropped_requests += 1
                continue
            try:
                self._queue.put_nowait((priority, self._order, frame))
            except queue.Full:
                if frame.kind == MessageKind.POLL:
                    self.dropped_polls += 1
                else:
                    self.dropped_requests += 1

    def close(self) -> None:
        if self._trace_sampler is not None:
            self._trace_sampler.close()
        self._stop_event.set()
        with self._trace_progress:
            self._trace_progress.notify_all()
        try:
            self._queue.put_nowait((-1, 0, None))
        except queue.Full:
            pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def wait_for_trace_drain(
        self,
        timeout_s: float,
        cancel_event: Optional[threading.Event] = None,
    ) -> bool:
        """Freeze sampling and wait until ground confirms the final cursor."""
        if timeout_s < 0:
            raise ValueError("timeout_s must not be negative")
        if self._trace_sampler is not None:
            self._trace_sampler.close()
        trace_session, latest_sample_seq = self._trace_buffer.latest_cursor()
        if latest_sample_seq == 0:
            return True

        deadline = time.monotonic() + timeout_s
        with self._trace_progress:
            while True:
                if (
                    self._observed_trace_session == trace_session
                    and self._observed_after_sample_seq >= latest_sample_seq
                ):
                    return True
                if self._stop_event.is_set() or (
                    cancel_event is not None and cancel_event.is_set()
                ):
                    return False
                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0:
                    return False
                self._trace_progress.wait(min(remaining_s, 0.1))

    @staticmethod
    def _priority(frame: Frame) -> int:
        if frame.kind == MessageKind.COMMAND:
            try:
                if decode_command(frame.payload).command_id == CommandId.TARGETED_STOP:
                    return 0
            except ValueError:
                pass
            return 10
        return 100

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                request = self._urgent_queue.get_nowait()
            except queue.Empty:
                try:
                    _, _, request = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue
            if request is None:
                return
            try:
                reply = self._handle(request)
            except ProtocolError:
                continue
            if reply is None:
                continue
            if self._wait(self._timing.turnaround_s):
                return
            if self._stop_event.is_set():
                return
            self._writer(reply)

    def _handle(self, request: Frame) -> Optional[bytes]:
        if request.session != self._ground_session:
            self._ground_session = request.session
            self._cache.begin_ground_session(request.session)
        if request.kind == MessageKind.POLL:
            return self._report(request)
        if request.kind == MessageKind.MAP_REQUEST:
            return self._map_report(request)
        if request.kind == MessageKind.PATH_REQUEST:
            return self._path_report(request)
        if request.kind == MessageKind.TRACE_REQUEST:
            trace_request = decode_trace_request(request.payload)
            self._observe_trace_cursor(trace_request)
            cached = self._cache.get(request.session, request.seq)
            if cached is not None:
                return cached
            reply = self._trace_report(request, trace_request)
            self._cache.put(request.session, request.seq, reply)
            return reply
        if request.kind != MessageKind.COMMAND:
            return None
        cached = self._cache.get(request.session, request.seq)
        if cached is not None:
            return cached
        reply = self._command(request)
        self._cache.put(request.session, request.seq, reply)
        return reply

    def _frame(self, kind: int, payload: bytes) -> bytes:
        return pack_frame(
            Frame(
                VERSION,
                NodeId.CAR,
                NodeId.GROUND,
                kind,
                0,
                self._session,
                self._seq.next(),
                payload,
            )
        )

    def _report(self, request: Frame) -> bytes:
        state = self._state_provider()
        payload = ReportPayload(
            request.session,
            request.seq,
            state.node_flags,
            state.uptime_ms,
            state.x_cm,
            state.y_cm,
            0,
            state.heading_cdeg,
            state.vx_cm_s,
            state.vy_cm_s,
            0,
            state.battery_cV,
            state.operation_state,
            state.pose_quality,
            self._active_command_seq,
            self._active_command_status,
            self._error_code,
        )
        return self._frame(MessageKind.REPORT, encode_report(payload))

    def _command(self, request: Frame) -> bytes:
        command_id = 0
        self._active_command_seq = request.seq
        self._active_command_status = int(AckStatus.RECEIVED)
        self._error_code = 0
        try:
            command = decode_command(request.payload)
            command_id = command.command_id
            if command.command_flags:
                raise ValueError("unknown command flags")
            if command_id == CommandId.PING:
                if command.command_body:
                    raise ValueError("PING body must be empty")
                result = CommandResult(AckStatus.COMPLETED)
            elif command_id == CommandId.TARGETED_STOP:
                if command.command_body:
                    raise ValueError("TARGETED_STOP body must be empty")
                result = self._on_stop()
            elif command_id == CommandId.SET_COORDINATE_FRAME:
                result = self._on_set_coordinate_frame(
                    decode_coordinate_frame(command.command_body)
                )
            elif command_id == CommandId.CAR_NAVIGATE_TO:
                result = self._on_navigate(
                    decode_car_navigate(command.command_body)
                )
            elif command_id == CommandId.CAR_DISASTER_RESCUE:
                if self._on_disaster_rescue is None:
                    result = CommandResult(
                        AckStatus.REJECTED, AckReason.UNSUPPORTED
                    )
                else:
                    result = self._on_disaster_rescue(
                        decode_disaster_rescue(command.command_body)
                    )
            elif command_id == CommandId.CAR_START_MAPPING:
                if command.command_body:
                    raise ValueError("CAR_START_MAPPING body must be empty")
                if self._on_start_mapping is None:
                    result = CommandResult(
                        AckStatus.REJECTED, AckReason.UNSUPPORTED
                    )
                else:
                    result = self._on_start_mapping(request.seq)
            elif command_id in (CommandId.CAR_ALARM_ON, CommandId.CAR_ALARM_OFF):
                if command.command_body:
                    raise ValueError("CAR_ALARM command body must be empty")
                if self._on_set_alarm is None:
                    result = CommandResult(
                        AckStatus.REJECTED, AckReason.UNSUPPORTED
                    )
                else:
                    result = self._on_set_alarm(
                        command_id == CommandId.CAR_ALARM_ON
                    )
            elif command_id == CommandId.CAR_START_MISSION:
                if command.command_body:
                    raise ValueError("CAR_START_MISSION body must be empty")
                if self._on_start_mission is None:
                    result = CommandResult(
                        AckStatus.REJECTED, AckReason.UNSUPPORTED
                    )
                else:
                    result = self._on_start_mission()
            else:
                result = CommandResult(AckStatus.REJECTED, AckReason.UNSUPPORTED)
        except ValueError as exc:
            result = CommandResult(
                AckStatus.REJECTED, AckReason.BAD_PAYLOAD, str(exc)
            )
        if self._active_command_status not in (
            int(AckStatus.COMPLETED),
            int(AckStatus.FAILED),
        ):
            self._active_command_status = int(result.status)
            self._error_code = (
                int(result.reason)
                if result.status in (AckStatus.REJECTED, AckStatus.FAILED)
                else 0
            )
        ack = AckPayload(
            request.session,
            request.seq,
            command_id,
            result.status,
            result.reason,
            result.detail,
        )
        return self._frame(MessageKind.ACK, encode_ack(ack))

    def _map_report(self, request: Frame) -> bytes:
        state = self._state_provider()
        payload = MapReportPayload(
            request.session,
            request.seq,
            state.map_revision,
            state.field_corners,
        )
        return self._frame(MessageKind.MAP_REPORT, encode_map_report(payload))

    def _path_report(self, request: Frame) -> bytes:
        state = self._state_provider()
        points = state.path_points
        max_points = (220 - 11) // 8
        if len(points) > max_points:
            step = float(len(points) - 1) / float(max_points - 1)
            points = tuple(points[round(index * step)] for index in range(max_points))
        payload = PathReportPayload(
            request.session,
            request.seq,
            state.path_revision,
            points,
        )
        return self._frame(MessageKind.PATH_REPORT, encode_path_report(payload))

    def _observe_trace_cursor(self, trace_request: TraceRequestPayload) -> None:
        trace_session, _ = self._trace_buffer.latest_cursor()
        if trace_request.known_trace_session != trace_session:
            return
        with self._trace_progress:
            if self._observed_trace_session != trace_session:
                self._observed_trace_session = trace_session
                self._observed_after_sample_seq = 0
            self._observed_after_sample_seq = max(
                self._observed_after_sample_seq,
                trace_request.after_sample_seq,
            )
            self._trace_progress.notify_all()

    def _trace_report(
        self,
        request: Frame,
        trace_request: TraceRequestPayload,
    ) -> bytes:
        report = self._trace_buffer.build_report(
            request.session,
            request.seq,
            trace_request,
        )
        return self._frame(
            MessageKind.TRACE_REPORT,
            encode_trace_report(report),
        )
