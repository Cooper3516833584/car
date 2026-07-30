#!/usr/bin/env python3
"""Start exactly one competition task from serial-screen button messages.

The serial screen may send its text with framing bytes or a line ending.  This
program therefore searches the received byte stream for the ASCII tokens
``MISSION1`` and ``MISSION2`` rather than requiring a particular line format.
``MISSION1`` is acknowledged immediately with three fast alarm pulses and
launched at once; ``MISSION2`` retains the delayed-selection behavior.
"""

from __future__ import annotations

import argparse
import logging
import os
import select
import signal
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Final

from components.sound_light_alarm import AlarmGPIOError, SoundLightAlarm

try:
    import termios
except ModuleNotFoundError:  # Allows pure scheduling tests on non-Linux hosts.
    termios = None  # type: ignore[assignment]


LOG = logging.getLogger("mission_screen_launcher")
DEFAULT_PORT: Final[str] = (
    "/dev/serial/by-id/usb-jixin.pro_CMSIS-DAP_LU_LU_2022_8888-if00"
)
DEFAULT_BAUDRATE: Final[int] = 9600
TOKEN_TO_TASK: Final[dict[bytes, str]] = {
    b"MISSION1": "main_task1.py",
    b"MISSION2": "main_task2.py",
}
MISSION1_FAST_BEEP_COUNT: Final[int] = 3
MISSION1_FAST_BEEP_ON_S: Final[float] = 0.15
MISSION1_FAST_BEEP_OFF_S: Final[float] = 0.10


def configure_serial(port: str, baudrate: int) -> int:
    """Open *port* for input only and configure it as raw 8N1."""

    if termios is None:
        raise RuntimeError("serial screen I/O requires Linux termios")
    baud = getattr(termios, f"B{baudrate}", None)
    if baud is None:
        raise ValueError(f"unsupported baud rate: {baudrate}")
    # This CDC-ACM serial screen starts its button-event stream only after a
    # normal bidirectional serial open.  The launcher never writes to the fd.
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CREAD | termios.CLOCAL | termios.CS8
    attrs[3] = 0
    attrs[4] = baud
    attrs[5] = baud
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    # Do not treat events generated before the service started as a new button
    # press.  This matters when the screen was tapped during boot.
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


@dataclass
class PendingLaunch:
    task_path: Path
    due_at: float
    prelaunch_alarm: bool = True
    alarm: SoundLightAlarm | None = None
    alarm_attempted: bool = False


class MissionScreenLauncher:
    """Serial token recognizer and single-child task supervisor."""

    def __init__(
        self, task_directory: Path, delay_s: float, alarm_duration_s: float = 5.0
    ) -> None:
        self.task_directory = task_directory
        self.delay_s = delay_s
        self.alarm_duration_s = alarm_duration_s
        self.pending: PendingLaunch | None = None
        self.child: subprocess.Popen[bytes] | None = None
        self._buffer: deque[int] = deque(maxlen=max(map(len, TOKEN_TO_TASK)))

    def receive(self, data: bytes, now: float) -> None:
        """Accept raw serial bytes and schedule the recognized mission."""

        for value in data.upper():
            self._buffer.append(value)
            window = bytes(self._buffer)
            for token, task_name in TOKEN_TO_TASK.items():
                if window.endswith(token):
                    if token == b"MISSION1":
                        self._sound_mission1_acknowledgement()
                        self.schedule(
                            self.task_directory / task_name,
                            now,
                            delay_s=0.0,
                            prelaunch_alarm=False,
                        )
                    else:
                        self.schedule(self.task_directory / task_name, now)
                    self._buffer.clear()  # One token must produce one launch.
                    return

    def schedule(
        self,
        task_path: Path,
        now: float,
        *,
        delay_s: float | None = None,
        prelaunch_alarm: bool = True,
    ) -> None:
        if self.child is not None and self.child.poll() is None:
            LOG.warning("ignored %s: %s is already running", task_path.name, self.child.args)
            return
        self._silence_pending_alarm()
        launch_delay = self.delay_s if delay_s is None else float(delay_s)
        self.pending = PendingLaunch(
            task_path,
            now + launch_delay,
            prelaunch_alarm=prelaunch_alarm,
        )
        LOG.info(
            "received %s; launching in %.1f s",
            task_path.stem.upper(),
            launch_delay,
        )

    def _sound_mission1_acknowledgement(self) -> None:
        alarm = None
        try:
            alarm = SoundLightAlarm()
            if not alarm.is_initialized:
                alarm.initialize()
            for index in range(MISSION1_FAST_BEEP_COUNT):
                alarm.on()
                time.sleep(MISSION1_FAST_BEEP_ON_S)
                alarm.off()
                if index + 1 < MISSION1_FAST_BEEP_COUNT:
                    time.sleep(MISSION1_FAST_BEEP_OFF_S)
            LOG.info("MISSION1 acknowledged with three fast alarm pulses")
        except AlarmGPIOError as exc:
            LOG.warning("MISSION1 acknowledgement alarm unavailable: %s", exc)
        finally:
            self._silence_alarm(alarm)

    def poll(self, now: float) -> None:
        if self.child is not None and self.child.poll() is not None:
            LOG.info("task exited with status %s", self.child.returncode)
            self.child = None
        if self.pending is None:
            return
        pending = self.pending
        alarm_at = pending.due_at - self.alarm_duration_s
        if (
            pending.prelaunch_alarm
            and not pending.alarm_attempted
            and now >= alarm_at
        ):
            pending.alarm_attempted = True
            try:
                alarm = SoundLightAlarm()
                if not alarm.is_initialized:
                    alarm.initialize()
                alarm.on()
                pending.alarm = alarm
                LOG.info(
                    "alarm sounding for %.1f s before %s",
                    self.alarm_duration_s,
                    pending.task_path.name,
                )
            except AlarmGPIOError as exc:
                LOG.warning(
                    "alarm unavailable; continuing with %s: %s",
                    pending.task_path.name,
                    exc,
                )
        if now < pending.due_at:
            return
        self.pending = None
        self._silence_alarm(pending.alarm)
        if not pending.task_path.is_file():
            LOG.error("cannot launch missing task file: %s", pending.task_path)
            return
        self.child = subprocess.Popen(
            [sys.executable, str(pending.task_path)],
            cwd=self.task_directory,
            start_new_session=True,
        )
        LOG.info("started %s (pid=%d)", pending.task_path.name, self.child.pid)

    def stop(self) -> None:
        self._silence_pending_alarm()
        if self.child is not None and self.child.poll() is None:
            LOG.info("stopping task process group %d", self.child.pid)
            os.killpg(self.child.pid, signal.SIGTERM)

    def _silence_pending_alarm(self) -> None:
        if self.pending is not None:
            self._silence_alarm(self.pending.alarm)

    @staticmethod
    def _silence_alarm(alarm: SoundLightAlarm | None) -> None:
        if alarm is None:
            return
        try:
            alarm.off()
        except AlarmGPIOError as exc:
            LOG.warning("could not silence alarm: %s", exc)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--delay-s", type=float, default=10.0)
    parser.add_argument("--alarm-duration-s", type=float, default=5.0)
    parser.add_argument(
        "--startup-quiet-s", type=float, default=1.0,
        help="require this much quiet serial input before accepting a button",
    )
    parser.add_argument("--task-directory", type=Path, default=Path(__file__).resolve().parent)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.delay_s < 0:
        raise ValueError("--delay-s must be non-negative")
    if not 0 <= args.alarm_duration_s <= args.delay_s:
        raise ValueError("--alarm-duration-s must be within --delay-s")
    if args.startup_quiet_s < 0:
        raise ValueError("--startup-quiet-s must be non-negative")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    launcher = MissionScreenLauncher(
        args.task_directory.resolve(), args.delay_s, args.alarm_duration_s
    )
    stopping = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    while not stopping:
        try:
            fd = configure_serial(args.port, args.baudrate)
            LOG.info("listening on %s at %d 8N1", args.port, args.baudrate)
            armed_at = time.monotonic() + args.startup_quiet_s
            armed = False
            try:
                while not stopping:
                    readable, _, _ = select.select([fd], [], [], 0.2)
                    now = time.monotonic()
                    if readable:
                        data = os.read(fd, 256)
                        if data:
                            if now < armed_at:
                                # A screen can replay button bytes while its
                                # USB-serial connection is being established.
                                armed_at = now + args.startup_quiet_s
                            else:
                                launcher.receive(data, now)
                    if not armed and now >= armed_at:
                        armed = True
                        LOG.info("serial input settled; awaiting a new button press")
                    if armed:
                        launcher.poll(now)
            finally:
                os.close(fd)
        except (OSError, ValueError) as exc:
            LOG.error("serial screen unavailable: %s; retrying", exc)
            for _ in range(10):
                if stopping:
                    break
                launcher.poll(time.monotonic())
                time.sleep(0.2)
    launcher.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
