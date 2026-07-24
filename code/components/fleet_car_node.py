"""FleetBus V1 car node with a parse-only RX callback and one reply worker."""

import queue
import threading
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
)
from .fleet_protocol import (
    FrameParser,
    RecentResponseCache,
    SequenceCounter,
    VERSION,
    decode_car_navigate,
    decode_command,
    decode_coordinate_frame,
    encode_ack,
    encode_map_report,
    encode_path_report,
    encode_report,
    new_session,
    pack_frame,
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
        timing: NodeTiming = NodeTiming(),
        wait: Optional[Callable[[float], bool]] = None,
    ) -> None:
        self._writer = writer
        self._state_provider = state_provider
        self._on_set_coordinate_frame = on_set_coordinate_frame
        self._on_navigate = on_navigate
        self._on_stop = on_stop
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

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
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
        self._stop_event.set()
        try:
            self._queue.put_nowait((-1, 0, None))
        except queue.Full:
            pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)

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
            reply = self._handle(request)
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
            else:
                result = CommandResult(AckStatus.REJECTED, AckReason.UNSUPPORTED)
        except ValueError as exc:
            result = CommandResult(
                AckStatus.REJECTED, AckReason.BAD_PAYLOAD, str(exc)
            )
        self._active_command_seq = request.seq
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
