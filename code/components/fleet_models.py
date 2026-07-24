"""Pure data models shared by the FleetBus V1 protocol layer."""

from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import Optional, Tuple


class NodeId(IntEnum):
    GROUND = 0x01
    DRONE = 0x10
    CAR = 0x20
    BROADCAST = 0xFF


class MessageKind(IntEnum):
    POLL = 0x01
    REPORT = 0x02
    COMMAND = 0x03
    ACK = 0x04
    MAP_REQUEST = 0x05
    MAP_REPORT = 0x06
    PATH_REQUEST = 0x07
    PATH_REPORT = 0x08


class CommandId(IntEnum):
    PING = 0x01
    TARGETED_STOP = 0x02
    SET_COORDINATE_FRAME = 0x10
    CAR_NAVIGATE_TO = 0x11
    DRONE_GOTO = 0x20
    DRONE_HOLD = 0x21
    CANCEL_TASK = 0x22


class AckStatus(IntEnum):
    RECEIVED = 1
    ACCEPTED = 2
    REJECTED = 3
    COMPLETED = 4
    FAILED = 5


class AckReason(IntEnum):
    NONE = 0
    BAD_PAYLOAD = 1
    NOT_READY = 2
    BUSY = 3
    OUTSIDE_FIELD = 4
    UNSUPPORTED = 5
    LINK_STATE_CHANGED = 6
    LOCALIZATION_INVALID = 7
    ALREADY_SYNCHRONIZED = 8
    INTERNAL_ERROR = 9


class PollFlags(IntFlag):
    REQUEST_BASIC_STATE = 0x0001
    REQUEST_HEALTH = 0x0002
    REQUEST_ACTIVE_COMMAND = 0x0004


DEFAULT_POLL_FLAGS = (
    PollFlags.REQUEST_BASIC_STATE
    | PollFlags.REQUEST_HEALTH
    | PollFlags.REQUEST_ACTIVE_COMMAND
)


class NodeFlags(IntFlag):
    POSE_VALID = 0x0001
    READY = 0x0002
    BUSY = 0x0004
    COORDINATE_FRAME_SYNCED = 0x0008
    ARMED_OR_MOTOR_ACTIVE = 0x0010
    LOCALIZATION_DEGRADED = 0x0020
    MAP_READY = 0x0040


class GoalFlags(IntFlag):
    HAS_FINAL_HEADING = 0x01


@dataclass(frozen=True)
class Frame:
    version: int
    src: int
    dst: int
    kind: int
    flags: int
    session: int
    seq: int
    payload: bytes = b""


@dataclass
class ParserStats:
    discarded_bytes: int = 0
    crc_failures: int = 0
    tail_failures: int = 0
    version_failures: int = 0
    oversize_frames: int = 0
    address_drops: int = 0


@dataclass(frozen=True)
class PollPayload:
    request_flags: int = int(DEFAULT_POLL_FLAGS)


@dataclass(frozen=True)
class ReportPayload:
    request_session: int
    request_seq: int
    node_flags: int
    node_uptime_ms: int
    x_cm: int
    y_cm: int
    z_cm: int
    heading_cdeg: int
    vx_cm_s: int
    vy_cm_s: int
    vz_cm_s: int
    battery_cV: int
    operation_state: int
    pose_quality: int
    active_command_seq: int
    active_command_status: int
    error_code: int


@dataclass(frozen=True)
class CommandPayload:
    command_id: int
    command_flags: int = 0
    command_body: bytes = b""


@dataclass(frozen=True)
class AckPayload:
    request_session: int
    request_seq: int
    command_id: int
    status: int
    reason: int = int(AckReason.NONE)
    detail: str = ""


@dataclass(frozen=True)
class CoordinateFrameCommand:
    origin_x_cm: int
    origin_y_cm: int
    startup_x_heading_cdeg: int


@dataclass(frozen=True)
class CarNavigateCommand:
    x_cm: int
    y_cm: int
    heading_cdeg: Optional[int] = None


@dataclass(frozen=True)
class DroneGotoCommand:
    x_cm: int
    y_cm: int
    z_cm: int
    heading_cdeg: Optional[int] = None


@dataclass(frozen=True)
class MapReportPayload:
    request_session: int
    request_seq: int
    map_revision: int
    corners: Tuple[Tuple[int, int], ...]


@dataclass(frozen=True)
class PathReportPayload:
    request_session: int
    request_seq: int
    path_revision: int
    points: Tuple[Tuple[int, int], ...]


@dataclass(frozen=True)
class NodeTiming:
    turnaround_s: float = 0.20
    queue_size: int = 16


@dataclass(frozen=True)
class CommandResult:
    status: int
    reason: int = int(AckReason.NONE)
    detail: str = ""


@dataclass(frozen=True)
class CarFleetState:
    node_flags: int
    uptime_ms: int
    x_cm: int
    y_cm: int
    heading_cdeg: int
    vx_cm_s: int = 0
    vy_cm_s: int = 0
    battery_cV: int = 0
    operation_state: int = 0
    pose_quality: int = 0
    active_command_seq: int = 0
    active_command_status: int = 0
    error_code: int = 0
    map_revision: int = 0
    field_corners: Tuple[Tuple[int, int], ...] = ()
    path_revision: int = 0
    path_points: Tuple[Tuple[int, int], ...] = ()
