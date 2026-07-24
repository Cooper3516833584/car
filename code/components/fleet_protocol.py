"""Hardware-free FleetBus V1 framing and payload codecs."""

from collections import OrderedDict
import secrets
import struct
from typing import List, Optional, Tuple

from .fleet_models import (
    AckPayload, CarNavigateCommand, CommandPayload, CoordinateFrameCommand,
    DroneGotoCommand, Frame, GoalFlags, MapReportPayload, ParserStats,
    PathReportPayload, PollPayload, ReportPayload,
)

MAGIC = b"\xD3\x91"
TAIL = b"\x1D\x0F"
VERSION = 1
MAX_PAYLOAD_LEN = 220
MAX_INNER_FRAME_LEN = 239
HEADER = struct.Struct("<2sBBBBBIHH")
CRC = struct.Struct("<H")
POLL = struct.Struct("<H")
REPORT = struct.Struct("<IHHIiiiHhhhHBBHBB")
ACK_HEADER = struct.Struct("<IHBBBB")
COORDINATE_FRAME = struct.Struct("<iiH")
CAR_NAVIGATE = struct.Struct("<Bii")
DRONE_GOTO = struct.Struct("<Biii")
HEADING = struct.Struct("<H")
POINT_REPORT_HEADER = struct.Struct("<IHIB")
POINT = struct.Struct("<ii")
FIXED_HEADER_LEN = HEADER.size
FRAME_OVERHEAD = FIXED_HEADER_LEN + CRC.size + len(TAIL)


class ProtocolError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def _range(name: str, value: int, low: int, high: int) -> int:
    if not isinstance(value, int) or not low <= value <= high:
        raise ProtocolError("range", "{} must be in {}..{}".format(name, low, high))
    return value


def _u8(name: str, value: int) -> int:
    return _range(name, value, 0, 0xFF)


def _u16(name: str, value: int) -> int:
    return _range(name, value, 0, 0xFFFF)


def _u32(name: str, value: int) -> int:
    return _range(name, value, 0, 0xFFFFFFFF)


def _i16(name: str, value: int) -> int:
    return _range(name, value, -0x8000, 0x7FFF)


def _i32(name: str, value: int) -> int:
    return _range(name, value, -0x80000000, 0x7FFFFFFF)


def _heading(value: int) -> int:
    return _range("heading_cdeg", value, 0, 35999)


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in bytes(data):
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def new_session() -> int:
    return secrets.randbits(32)


class SequenceCounter:
    def __init__(self, initial: int = 0) -> None:
        self._value = _u16("initial", initial)

    def next(self) -> int:
        self._value = (self._value % 0xFFFF) + 1
        return self._value


class RecentResponseCache:
    def __init__(self, max_items: int = 64) -> None:
        if max_items <= 0:
            raise ValueError("max_items must be positive")
        self._max_items = max_items
        self._ground_session = None  # type: Optional[int]
        self._items = OrderedDict()  # type: OrderedDict[Tuple[int, int], bytes]

    def begin_ground_session(self, session: int) -> None:
        session = _u32("session", session)
        if session != self._ground_session:
            self._ground_session = session
            self._items.clear()

    def get(self, session: int, seq: int) -> Optional[bytes]:
        key = (_u32("session", session), _u16("seq", seq))
        value = self._items.get(key)
        if value is not None:
            self._items.move_to_end(key)
        return value

    def put(self, session: int, seq: int, response: bytes) -> None:
        self.begin_ground_session(session)
        key = (session, _u16("seq", seq))
        self._items[key] = bytes(response)
        self._items.move_to_end(key)
        while len(self._items) > self._max_items:
            self._items.popitem(last=False)


def pack_frame(frame: Frame) -> bytes:
    payload = bytes(frame.payload)
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ProtocolError("oversize", "payload exceeds FleetBus V1 limit")
    header = HEADER.pack(
        MAGIC, _u8("version", frame.version), _u8("src", frame.src),
        _u8("dst", frame.dst), _u8("kind", frame.kind), _u8("flags", frame.flags),
        _u32("session", frame.session), _u16("seq", frame.seq), len(payload),
    )
    packed = header + payload + CRC.pack(crc16_ccitt_false(header[2:] + payload)) + TAIL
    if len(packed) > MAX_INNER_FRAME_LEN:
        raise ProtocolError("oversize", "inner frame exceeds FleetBus V1 limit")
    return packed


def unpack_frame(data: bytes) -> Frame:
    data = bytes(data)
    if len(data) < FRAME_OVERHEAD:
        raise ProtocolError("truncated", "frame is too short")
    magic, version, src, dst, kind, flags, session, seq, size = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ProtocolError("magic", "invalid FleetBus magic")
    if size > MAX_PAYLOAD_LEN:
        raise ProtocolError("oversize", "payload exceeds FleetBus V1 limit")
    if len(data) != FRAME_OVERHEAD + size:
        raise ProtocolError("length", "frame length does not match payload_len")
    if data[-2:] != TAIL:
        raise ProtocolError("tail", "invalid FleetBus tail")
    end = FIXED_HEADER_LEN + size
    if CRC.unpack_from(data, end)[0] != crc16_ccitt_false(data[2:end]):
        raise ProtocolError("crc", "FleetBus CRC mismatch")
    if version != VERSION:
        raise ProtocolError("version", "unsupported FleetBus version")
    return Frame(version, src, dst, kind, flags, session, seq, data[FIXED_HEADER_LEN:end])


class FrameParser:
    def __init__(self, local_node: Optional[int] = None) -> None:
        self._buffer = bytearray()
        self._local_node = None if local_node is None else _u8("local_node", local_node)
        self.stats = ParserStats()

    def feed(self, data: bytes) -> List[Frame]:
        self._buffer.extend(data)
        frames = []  # type: List[Frame]
        while True:
            start = self._buffer.find(MAGIC)
            if start < 0:
                keep = 1 if self._buffer.endswith(MAGIC[:1]) else 0
                self.stats.discarded_bytes += len(self._buffer) - keep
                if keep:
                    del self._buffer[:-keep]
                else:
                    self._buffer.clear()
                return frames
            if start:
                self.stats.discarded_bytes += start
                del self._buffer[:start]
            if len(self._buffer) < FIXED_HEADER_LEN:
                return frames
            size = int.from_bytes(self._buffer[13:15], "little")
            if size > MAX_PAYLOAD_LEN:
                self.stats.oversize_frames += 1
                self._drop()
                continue
            total = FRAME_OVERHEAD + size
            if len(self._buffer) < total:
                return frames
            try:
                frame = unpack_frame(bytes(self._buffer[:total]))
            except ProtocolError as exc:
                if exc.code == "crc":
                    self.stats.crc_failures += 1
                elif exc.code == "tail":
                    self.stats.tail_failures += 1
                elif exc.code == "version":
                    self.stats.version_failures += 1
                self._drop()
                continue
            del self._buffer[:total]
            if self._local_node is not None and frame.dst != self._local_node:
                self.stats.address_drops += 1
            else:
                frames.append(frame)

    def _drop(self) -> None:
        del self._buffer[0]
        self.stats.discarded_bytes += 1


def encode_poll(value: PollPayload) -> bytes:
    return POLL.pack(_u16("request_flags", value.request_flags))


def decode_poll(data: bytes) -> PollPayload:
    if len(data) != POLL.size:
        raise ProtocolError("payload", "POLL payload must be 2 bytes")
    return PollPayload(POLL.unpack(data)[0])


def encode_report(value: ReportPayload) -> bytes:
    return REPORT.pack(
        _u32("request_session", value.request_session), _u16("request_seq", value.request_seq),
        _u16("node_flags", value.node_flags), _u32("node_uptime_ms", value.node_uptime_ms),
        _i32("x_cm", value.x_cm), _i32("y_cm", value.y_cm), _i32("z_cm", value.z_cm),
        _heading(value.heading_cdeg), _i16("vx_cm_s", value.vx_cm_s),
        _i16("vy_cm_s", value.vy_cm_s), _i16("vz_cm_s", value.vz_cm_s),
        _u16("battery_cV", value.battery_cV), _u8("operation_state", value.operation_state),
        _range("pose_quality", value.pose_quality, 0, 4),
        _u16("active_command_seq", value.active_command_seq),
        _u8("active_command_status", value.active_command_status), _u8("error_code", value.error_code),
    )


def decode_report(data: bytes) -> ReportPayload:
    if len(data) != REPORT.size:
        raise ProtocolError("payload", "REPORT payload has invalid length")
    value = ReportPayload(*REPORT.unpack(data))
    _heading(value.heading_cdeg)
    _range("pose_quality", value.pose_quality, 0, 4)
    return value


def encode_command(value: CommandPayload) -> bytes:
    return bytes((_u8("command_id", value.command_id), _u8("command_flags", value.command_flags))) + bytes(value.command_body)


def decode_command(data: bytes) -> CommandPayload:
    if len(data) < 2:
        raise ProtocolError("payload", "COMMAND payload is too short")
    return CommandPayload(data[0], data[1], bytes(data[2:]))


def encode_ack(value: AckPayload) -> bytes:
    detail = value.detail.encode("utf-8")
    data = ACK_HEADER.pack(_u32("request_session", value.request_session), _u16("request_seq", value.request_seq), _u8("command_id", value.command_id), _u8("status", value.status), _u8("reason", value.reason), _u8("detail_len", len(detail))) + detail
    if len(data) > MAX_PAYLOAD_LEN:
        raise ProtocolError("oversize", "ACK payload exceeds FleetBus V1 limit")
    return data


def decode_ack(data: bytes) -> AckPayload:
    if len(data) < ACK_HEADER.size:
        raise ProtocolError("payload", "ACK payload is too short")
    values = ACK_HEADER.unpack_from(data)
    if len(data) != ACK_HEADER.size + values[-1]:
        raise ProtocolError("payload", "ACK detail length mismatch")
    try:
        detail = data[ACK_HEADER.size:].decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ProtocolError("payload", "ACK detail is not UTF-8") from exc
    return AckPayload(*values[:-1], detail=detail)


def encode_coordinate_frame(value: CoordinateFrameCommand) -> bytes:
    return COORDINATE_FRAME.pack(_i32("origin_x_cm", value.origin_x_cm), _i32("origin_y_cm", value.origin_y_cm), _heading(value.startup_x_heading_cdeg))


def decode_coordinate_frame(data: bytes) -> CoordinateFrameCommand:
    if len(data) != COORDINATE_FRAME.size:
        raise ProtocolError("payload", "coordinate frame body has invalid length")
    value = CoordinateFrameCommand(*COORDINATE_FRAME.unpack(data))
    _heading(value.startup_x_heading_cdeg)
    return value


def _goal(codec: struct.Struct, values: Tuple[int, ...], heading: Optional[int]) -> bytes:
    data = codec.pack(0, *values)
    return data if heading is None else bytes((int(GoalFlags.HAS_FINAL_HEADING),)) + data[1:] + HEADING.pack(_heading(heading))


def _decode_goal(data: bytes, codec: struct.Struct) -> Tuple[Tuple[int, ...], Optional[int]]:
    if len(data) not in (codec.size, codec.size + HEADING.size):
        raise ProtocolError("payload", "goal body has invalid length")
    unpacked = codec.unpack_from(data)
    flags, values = unpacked[0], unpacked[1:]
    if flags & ~int(GoalFlags.HAS_FINAL_HEADING):
        raise ProtocolError("payload", "goal contains unknown flags")
    present = bool(flags & int(GoalFlags.HAS_FINAL_HEADING))
    if present != (len(data) == codec.size + HEADING.size):
        raise ProtocolError("payload", "goal heading flag/length mismatch")
    heading = HEADING.unpack_from(data, codec.size)[0] if present else None
    if heading is not None:
        _heading(heading)
    return values, heading


def encode_car_navigate(value: CarNavigateCommand) -> bytes:
    return _goal(CAR_NAVIGATE, (_i32("x_cm", value.x_cm), _i32("y_cm", value.y_cm)), value.heading_cdeg)


def decode_car_navigate(data: bytes) -> CarNavigateCommand:
    values, heading = _decode_goal(data, CAR_NAVIGATE)
    return CarNavigateCommand(values[0], values[1], heading)


def encode_drone_goto(value: DroneGotoCommand) -> bytes:
    return _goal(DRONE_GOTO, (_i32("x_cm", value.x_cm), _i32("y_cm", value.y_cm), _i32("z_cm", value.z_cm)), value.heading_cdeg)


def decode_drone_goto(data: bytes) -> DroneGotoCommand:
    values, heading = _decode_goal(data, DRONE_GOTO)
    return DroneGotoCommand(values[0], values[1], values[2], heading)


def _encode_points(session: int, seq: int, revision: int, points: Tuple[Tuple[int, int], ...]) -> bytes:
    data = POINT_REPORT_HEADER.pack(_u32("request_session", session), _u16("request_seq", seq), _u32("revision", revision), _u8("point_count", len(points)))
    data += b"".join(POINT.pack(_i32("x_cm", x), _i32("y_cm", y)) for x, y in points)
    if len(data) > MAX_PAYLOAD_LEN:
        raise ProtocolError("oversize", "point report exceeds FleetBus V1 limit")
    return data


def _decode_points(data: bytes) -> Tuple[int, int, int, Tuple[Tuple[int, int], ...]]:
    if len(data) < POINT_REPORT_HEADER.size:
        raise ProtocolError("payload", "point report is too short")
    session, seq, revision, count = POINT_REPORT_HEADER.unpack_from(data)
    if len(data) != POINT_REPORT_HEADER.size + count * POINT.size:
        raise ProtocolError("payload", "point report count/length mismatch")
    points = tuple(POINT.unpack_from(data, POINT_REPORT_HEADER.size + index * POINT.size) for index in range(count))
    return session, seq, revision, points


def encode_map_report(value: MapReportPayload) -> bytes:
    if len(value.corners) not in (0, 4):
        raise ProtocolError("payload", "MAP_REPORT requires zero or four corners")
    return _encode_points(value.request_session, value.request_seq, value.map_revision, value.corners)


def decode_map_report(data: bytes) -> MapReportPayload:
    session, seq, revision, points = _decode_points(data)
    if len(points) not in (0, 4):
        raise ProtocolError("payload", "MAP_REPORT requires zero or four corners")
    return MapReportPayload(session, seq, revision, points)


def encode_path_report(value: PathReportPayload) -> bytes:
    return _encode_points(value.request_session, value.request_seq, value.path_revision, value.points)


def decode_path_report(data: bytes) -> PathReportPayload:
    session, seq, revision, points = _decode_points(data)
    return PathReportPayload(session, seq, revision, points)
