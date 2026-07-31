"""FleetBus V1 models shared with the ground station and drone."""

from dataclasses import dataclass
from enum import IntEnum

from components.fleet_models import (
    AckPayload, AckReason, AckStatus, CarNavigateCommand, CommandId,
    CommandPayload, Frame, MapReportPayload, MessageKind, NodeFlags, NodeId,
    NodeTiming, ParserStats, PathReportPayload, PollPayload, ReportPayload,
    MissionId,
)


class CarOperationState(IntEnum):
    STARTING = 0
    CALIBRATING = 1
    READY = 2
    PLANNING = 3
    FOLLOWING = 4
    FINAL_APPROACH = 5
    GEAR_CHANGE = 6
    ARRIVED = 7
    PAUSED = 8
    BLOCKED = 9
    LOCALIZATION_LOST = 10
    FAILED = 11
    CLOSED = 12
    MISSION1_REQUESTED = 13
    MISSION2_REQUESTED = 14


@dataclass(frozen=True)
class CarFleetState:
    node_flags: int
    node_uptime_ms: int
    x_cm: int = 0
    y_cm: int = 0
    z_cm: int = 0
    heading_cdeg: int = 0
    vx_cm_s: int = 0
    vy_cm_s: int = 0
    vz_cm_s: int = 0
    battery_cV: int = 0
    operation_state: int = int(CarOperationState.STARTING)
    pose_quality: int = 0
    error_code: int = 0
