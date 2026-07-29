"""Bounded FleetBus command queue with no vehicle-control side effects."""

from dataclasses import dataclass
import itertools
import queue
import threading
from typing import Optional

from .models import AckStatus, CommandId


@dataclass(frozen=True)
class CommandQueueStatus:
    active_command_seq: int = 0
    status: int = 0
    error_code: int = 0


class CarCommandQueue:
    def __init__(self, maxsize: int = 16) -> None:
        if maxsize <= 0:
            raise ValueError("maxsize must be positive")
        self._queue = queue.PriorityQueue(maxsize=maxsize)
        self._counter = itertools.count()
        self._lock = threading.Lock()
        self._status = CommandQueueStatus()

    @staticmethod
    def _priority(command) -> int:
        if int(getattr(command, "command_id")) == int(CommandId.TARGETED_STOP):
            return 0
        return 10

    @staticmethod
    def _sequence(command) -> int:
        return int(getattr(command, "seq", 0))

    def put(self, command) -> bool:
        try:
            self._queue.put_nowait(
                (self._priority(command), next(self._counter), command)
            )
        except queue.Full:
            return False
        return True

    def receive(self, timeout: Optional[float] = None):
        _, _, command = self._queue.get(timeout=timeout)
        return command

    def accept(self, command) -> None:
        with self._lock:
            self._status = CommandQueueStatus(
                self._sequence(command), int(AckStatus.ACCEPTED), 0
            )

    def complete(self, command) -> None:
        with self._lock:
            self._status = CommandQueueStatus(
                self._sequence(command), int(AckStatus.COMPLETED), 0
            )

    def fail(self, command, error_code) -> None:
        with self._lock:
            self._status = CommandQueueStatus(
                self._sequence(command), int(AckStatus.FAILED), int(error_code)
            )

    def status(self) -> CommandQueueStatus:
        with self._lock:
            return self._status
