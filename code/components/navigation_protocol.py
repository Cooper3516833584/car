#!/usr/bin/env python3
"""Authenticated GroundStationLink V2 commands for car navigation.

The HC-14 transport removes the ``BB 33`` envelope and hands this component one
complete inner ``AA 22`` frame.  This module preserves the existing V2 metadata,
checksum and HMAC format and adds only the car-specific ``NAVIGATE_TO`` payload.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import hashlib
import hmac
import os
import secrets
import struct
import threading
from typing import Callable, Final

from .navigation import NavigationGoal, normalize_heading_deg


MAGIC: Final[bytes] = b"\xAA\x22"
PROTOCOL_VERSION: Final[int] = 2
MESSAGE_TYPE_COMMAND: Final[int] = 4
MESSAGE_TYPE_COMMAND_ACK: Final[int] = 5
COMMAND_STOP_MISSION: Final[int] = 5
COMMAND_NAVIGATE_TO: Final[int] = 0x20
FLAG_HAS_HEADING: Final[int] = 0x01
HMAC_LEN: Final[int] = 8
MAX_PAYLOAD_LEN: Final[int] = 128
HEADER = struct.Struct("<2sBB")
METADATA = struct.Struct(">BBIH")
NAVIGATION_BASE = struct.Struct("<BBii")
HEADING = struct.Struct("<H")
ACK_PAYLOAD = struct.Struct(">BBHBB")


class NavigationProtocolError(ValueError):
    pass


class NavigationCommandRejected(RuntimeError):
    def __init__(self, reason: "RejectReason", message: str) -> None:
        super().__init__(message)
        self.reason = reason


class AckStatus(IntEnum):
    RECEIVED = 1
    ACCEPTED = 2
    REJECTED = 3
    COMPLETED = 4
    FAILED = 5


class RejectReason(IntEnum):
    NONE = 0
    BAD_PAYLOAD = 1
    TASK_BUSY = 4
    LINK_DOWN = 8
    UNKNOWN_COMMAND = 9


@dataclass(frozen=True, slots=True)
class AuthenticatedFrame:
    msg_type: int
    flags: int
    session: int
    seq: int
    payload: bytes


@dataclass(frozen=True, slots=True)
class NavigationCommandReceipt:
    session: int
    seq: int
    command_id: int


def load_navigation_hmac_key(
    env_name: str = "GROUND_STATION_HMAC_KEY_HEX",
) -> bytes:
    encoded = os.environ.get(env_name, "").strip()
    if not encoded:
        raise NavigationProtocolError(f"missing HMAC key environment variable {env_name}")
    try:
        key = bytes.fromhex(encoded)
    except ValueError as exc:
        raise NavigationProtocolError("HMAC key must be hexadecimal") from exc
    if len(key) < 16:
        raise NavigationProtocolError("HMAC key must contain at least 16 bytes")
    return key


def _tag(data: bytes, key: bytes) -> bytes:
    if len(key) < 16:
        raise NavigationProtocolError("HMAC key must contain at least 16 bytes")
    return hmac.new(key, data, hashlib.sha256).digest()[:HMAC_LEN]


def pack_authenticated_frame(
    msg_type: int,
    payload: bytes,
    *,
    session: int,
    seq: int,
    key: bytes,
    flags: int = 0,
) -> bytes:
    if len(payload) > MAX_PAYLOAD_LEN:
        raise NavigationProtocolError("payload exceeds 128 bytes")
    metadata = METADATA.pack(
        PROTOCOL_VERSION,
        flags & 0xFF,
        session & 0xFFFFFFFF,
        seq & 0xFFFF,
    )
    data_len = len(metadata) + len(payload) + HMAC_LEN
    header = HEADER.pack(MAGIC, msg_type & 0xFF, data_len)
    protected = header + metadata + bytes(payload)
    without_checksum = protected + _tag(protected, key)
    return without_checksum + bytes((sum(without_checksum) & 0xFF,))


def unpack_authenticated_frame(data: bytes, *, key: bytes) -> AuthenticatedFrame:
    frame = bytes(data)
    minimum = HEADER.size + METADATA.size + HMAC_LEN + 1
    if len(frame) < minimum:
        raise NavigationProtocolError("frame is too short")
    magic, msg_type, data_len = HEADER.unpack_from(frame)
    if magic != MAGIC:
        raise NavigationProtocolError("invalid AA 22 header")
    if len(frame) != HEADER.size + data_len + 1:
        raise NavigationProtocolError("frame length does not match data_len")
    if frame[-1] != sum(frame[:-1]) & 0xFF:
        raise NavigationProtocolError("frame checksum mismatch")
    tag_offset = len(frame) - 1 - HMAC_LEN
    if not hmac.compare_digest(frame[tag_offset:-1], _tag(frame[:tag_offset], key)):
        raise NavigationProtocolError("frame HMAC mismatch")
    version, flags, session, seq = METADATA.unpack_from(frame, HEADER.size)
    if version != PROTOCOL_VERSION:
        raise NavigationProtocolError("unsupported protocol version")
    payload_start = HEADER.size + METADATA.size
    return AuthenticatedFrame(msg_type, flags, session, seq, frame[payload_start:tag_offset])


def encode_navigation_payload(goal: NavigationGoal) -> bytes:
    flags = FLAG_HAS_HEADING if goal.final_heading_deg is not None else 0
    payload = NAVIGATION_BASE.pack(
        COMMAND_NAVIGATE_TO,
        flags,
        round(goal.x_cm),
        round(goal.y_cm),
    )
    if goal.final_heading_deg is not None:
        payload += HEADING.pack(round(normalize_heading_deg(goal.final_heading_deg) * 100) % 36000)
    return payload


def decode_navigation_payload(payload: bytes) -> NavigationGoal:
    if len(payload) not in (NAVIGATION_BASE.size, NAVIGATION_BASE.size + HEADING.size):
        raise NavigationProtocolError("NAVIGATE_TO payload has invalid length")
    command_id, flags, x_cm, y_cm = NAVIGATION_BASE.unpack_from(payload)
    if command_id != COMMAND_NAVIGATE_TO:
        raise NavigationProtocolError("payload is not NAVIGATE_TO")
    if flags & ~FLAG_HAS_HEADING:
        raise NavigationProtocolError("NAVIGATE_TO contains unknown flags")
    has_heading = bool(flags & FLAG_HAS_HEADING)
    if has_heading != (len(payload) == NAVIGATION_BASE.size + HEADING.size):
        raise NavigationProtocolError("NAVIGATE_TO heading flag/length mismatch")
    heading = None
    if has_heading:
        heading_raw = HEADING.unpack_from(payload, NAVIGATION_BASE.size)[0]
        if heading_raw >= 36000:
            raise NavigationProtocolError("NAVIGATE_TO heading is outside [0, 360)")
        heading = heading_raw / 100.0
    return NavigationGoal(float(x_cm), float(y_cm), heading)


def pack_navigation_command(
    goal: NavigationGoal,
    *,
    session: int,
    seq: int,
    key: bytes,
) -> bytes:
    return pack_authenticated_frame(
        MESSAGE_TYPE_COMMAND,
        encode_navigation_payload(goal),
        session=session,
        seq=seq,
        key=key,
    )


class GroundNavigationProtocol:
    """Validate navigation commands, deduplicate them and build V2 ACKs."""

    def __init__(
        self,
        *,
        key: bytes,
        on_goal: Callable[[NavigationGoal, NavigationCommandReceipt], None],
        on_stop: Callable[[NavigationCommandReceipt], None],
        cache_size: int = 256,
    ) -> None:
        if cache_size <= 0:
            raise ValueError("cache_size must be positive")
        _tag(b"key-check", key)
        self.key = bytes(key)
        self.on_goal = on_goal
        self.on_stop = on_stop
        self.cache_size = cache_size
        self._session = secrets.randbits(32)
        self._outbound_seq = 0
        self._cache: dict[tuple[int, int], tuple[bytes, ...]] = {}
        self._cache_order: list[tuple[int, int]] = []
        self._lock = threading.Lock()

    def handle_frame(self, data: bytes) -> tuple[bytes, ...]:
        frame = unpack_authenticated_frame(data, key=self.key)
        if frame.msg_type != MESSAGE_TYPE_COMMAND:
            raise NavigationProtocolError("only COMMAND frames are accepted")
        if not frame.payload:
            raise NavigationProtocolError("empty command payload")
        cache_key = (frame.session, frame.seq)
        with self._lock:
            cached = self._cache.get(cache_key)
        if cached is not None:
            return cached

        command_id = frame.payload[0]
        receipt = NavigationCommandReceipt(frame.session, frame.seq, command_id)
        replies = [self._ack(receipt, AckStatus.RECEIVED, RejectReason.NONE)]
        try:
            if command_id == COMMAND_NAVIGATE_TO:
                goal = decode_navigation_payload(frame.payload)
                self.on_goal(goal, receipt)
            elif command_id == COMMAND_STOP_MISSION:
                if len(frame.payload) != 1:
                    raise NavigationProtocolError("STOP_MISSION payload must be one byte")
                self.on_stop(receipt)
            else:
                replies.append(
                    self._ack(receipt, AckStatus.REJECTED, RejectReason.UNKNOWN_COMMAND)
                )
                return self._remember(cache_key, tuple(replies))
        except NavigationCommandRejected as exc:
            replies.append(self._ack(receipt, AckStatus.REJECTED, exc.reason))
            return self._remember(cache_key, tuple(replies))
        except (NavigationProtocolError, ValueError, RuntimeError):
            replies.append(self._ack(receipt, AckStatus.REJECTED, RejectReason.BAD_PAYLOAD))
            return self._remember(cache_key, tuple(replies))

        replies.append(self._ack(receipt, AckStatus.ACCEPTED, RejectReason.NONE))
        return self._remember(cache_key, tuple(replies))

    def build_status_ack(
        self,
        receipt: NavigationCommandReceipt,
        status: AckStatus,
        reason: RejectReason = RejectReason.NONE,
    ) -> bytes:
        return self._ack(receipt, status, reason)

    def _ack(
        self,
        receipt: NavigationCommandReceipt,
        status: AckStatus,
        reason: RejectReason,
    ) -> bytes:
        payload = ACK_PAYLOAD.pack(
            MESSAGE_TYPE_COMMAND,
            receipt.command_id,
            receipt.seq,
            status,
            reason,
        )
        with self._lock:
            seq = self._outbound_seq
            self._outbound_seq = (self._outbound_seq + 1) & 0xFFFF
        return pack_authenticated_frame(
            MESSAGE_TYPE_COMMAND_ACK,
            payload,
            session=self._session,
            seq=seq,
            key=self.key,
        )

    def _remember(
        self,
        key: tuple[int, int],
        replies: tuple[bytes, ...],
    ) -> tuple[bytes, ...]:
        with self._lock:
            self._cache[key] = replies
            self._cache_order.append(key)
            while len(self._cache_order) > self.cache_size:
                oldest = self._cache_order.pop(0)
                self._cache.pop(oldest, None)
        return replies
