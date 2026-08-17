#!/usr/bin/env python3
"""Start exactly one competition task from serial-screen button messages.

The serial screen may send its text with framing bytes or a line ending.  This
program therefore searches the received byte stream for the ASCII tokens
``MISSION1`` and ``MISSION2`` rather than requiring a particular line format.
Both missions are acknowledged immediately with three fast alarm pulses and
launched at once.

Every hardware parameter (screen port/baudrate, HC-14 port, alarm GPIO and the
runtime radar-centre state file) comes from the vehicle TOML profile.  The
launched task receives the same ``--config`` path so launcher and task can
never disagree about hardware.
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
from typing import Callable, Final

from components import SerialCommunicationDriver
from components.fleet_car_node import FleetCarNode
from components.fleet_models import (
    AckReason,
    AckStatus,
    CarFleetState,
    CommandResult,
)
from components.sound_light_alarm import AlarmGPIOError, SoundLightAlarm
from config.loader import load_car_config, resolve_config_path
from config.runtime_state import RuntimeRadarCenterState

try:
    import termios
except ModuleNotFoundError:  # Allows pure scheduling tests on non-Linux hosts.
    termios = None  # type: ignore[assignment]


LOG = logging.getLogger("mission_screen_launcher")
TOKEN_TO_TASK: Final[dict[bytes, str]] = {
    b"MISSION1": "main_task1.py",
    b"MISSION2": "main_task2.py",
}
MISSION_SELECTION_BEEP_COUNT: Final[int] = 3
MISSION_SELECTION_BEEP_ON_S: Final[float] = 0.15
MISSION_SELECTION_BEEP_OFF_S: Final[float] = 0.10


def _build_alarm(car_config) -> SoundLightAlarm:
    """Build the alarm from the profile's [hardware.alarm_gpio] section."""
    gpio = car_config.hardware.alarm_gpio
    return SoundLightAlarm(
        sysfs_gpio_root=gpio.sysfs_root,
        bank_label=gpio.bank_label,
        line_offset=gpio.line_offset,
        active_low=gpio.active_low,
    )


class IdleFleetReporter:
    """Answer ground-station polls while no competition task owns HC-14."""

    def __init__(self, distance_provider, port: str) -> None:
        self._started_at = time.monotonic()
        self._distance_provider = distance_provider
        self._link = SerialCommunicationDriver(
            port=port,
            on_bytes=self._on_frame,
            on_connected=lambda: LOG.info("idle FleetBus reporter connected"),
            on_disconnected=lambda error: LOG.warning(
                "idle FleetBus reporter disconnected: %s", error
            ) if error is not None else None,
        )
        self._node = FleetCarNode(
            writer=self._link.write,
            state_provider=self._state,
            on_set_coordinate_frame=self._unsupported,
            on_navigate=self._unsupported,
            on_stop=self._unsupported,
        )

    @staticmethod
    def _unsupported(*_args) -> CommandResult:
        return CommandResult(
            AckStatus.REJECTED,
            AckReason.NOT_READY,
            "competition task is not running",
        )

    def _state(self) -> CarFleetState:
        return CarFleetState(
            node_flags=0,
            uptime_ms=round(
                (time.monotonic() - self._started_at) * 1000.0
            ) & 0xFFFFFFFF,
            x_cm=0,
            y_cm=0,
            heading_cdeg=0,
            radar_center_behind_a_centi_cm=round(
                float(self._distance_provider()) * 100.0
            ),
        )

    def _on_frame(self, frame: bytes) -> None:
        self._node.feed_frame(frame)

    def start(self) -> None:
        self._node.start()
        self._link.start()

    def close(self) -> None:
        self._link.close()
        self._node.close()


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
        self,
        task_directory: Path,
        delay_s: float,
        alarm_duration_s: float = 5.0,
        config_path: Path | None = None,
        idle_reporter_factory: Callable | None = None,
    ) -> None:
        self.task_directory = task_directory
        self.delay_s = delay_s
        self.alarm_duration_s = alarm_duration_s
        self.pending: PendingLaunch | None = None
        self.child: subprocess.Popen[bytes] | None = None
        self.config_path = (
            resolve_config_path()
            if config_path is None
            else Path(config_path)
        )
        self.car_config = load_car_config(self.config_path)
        self.screen_port = self.car_config.devices.screen.port
        self.screen_baudrate = self.car_config.devices.screen.baudrate
        self.hc14_port = self.car_config.devices.hc14.port
        self._runtime_state = RuntimeRadarCenterState(
            self.car_config.runtime,
            base_directory=task_directory.parent,
        )
        self.radar_center_behind_a_cm = self._runtime_state.load(
            self.car_config.missions.common.radar_center_behind_a_cm
        )
        maximum_token_length = 1 + max(
            *(map(len, TOKEN_TO_TASK)),
            *(
                map(
                    len,
                    (
                        format(value, "g")
                        for value in (
                            self._runtime_state.config
                            .allowed_radar_center_behind_a_cm
                        )
                    ),
                )
            ),
        )
        self._buffer: deque[int] = deque(maxlen=maximum_token_length)
        self._idle_reporter_factory = idle_reporter_factory
        self._idle_reporter = None

    def start_idle_reporting(self) -> None:
        if self._idle_reporter is not None or self._idle_reporter_factory is None:
            return
        reporter = self._idle_reporter_factory(
            lambda: self.radar_center_behind_a_cm,
            self.hc14_port,
        )
        reporter.start()
        self._idle_reporter = reporter

    def _stop_idle_reporting(self) -> None:
        reporter, self._idle_reporter = self._idle_reporter, None
        if reporter is not None:
            reporter.close()

    def receive(self, data: bytes, now: float) -> None:
        """Accept raw serial bytes and schedule the recognized mission."""

        distance_tokens = {
            format(value, "g").encode("ascii"): float(value)
            for value in self._runtime_state.config.allowed_radar_center_behind_a_cm
        }
        for value in data.upper():
            self._buffer.append(value)
            window = bytes(self._buffer)
            for token, distance_cm in distance_tokens.items():
                preceding_index = len(window) - len(token) - 1
                has_numeric_prefix = (
                    preceding_index >= 0
                    and window[preceding_index] in b"0123456789."
                )
                if window.endswith(token) and not has_numeric_prefix:
                    if self.child is not None and self.child.poll() is None:
                        LOG.warning(
                            "ignored radar centre distance %g cm while task "
                            "is running",
                            distance_cm,
                        )
                        self._buffer.clear()
                        return
                    try:
                        selected = self._runtime_state.save(distance_cm)
                    except OSError as exc:
                        LOG.error(
                            "could not save radar centre distance: %s", exc
                        )
                    else:
                        self.radar_center_behind_a_cm = selected
                        LOG.info(
                            "radar centre distance behind A set to %g cm",
                            selected,
                        )
                    self._buffer.clear()
                    return
            for token, task_name in TOKEN_TO_TASK.items():
                if window.endswith(token):
                    self._sound_selection_acknowledgement(task_name)
                    self.schedule(
                        self.task_directory / task_name,
                        now,
                        delay_s=0.0,
                        prelaunch_alarm=False,
                    )
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

    def _sound_selection_acknowledgement(self, task_name: str) -> None:
        alarm = None
        try:
            alarm = _build_alarm(self.car_config)
            if not alarm.is_initialized:
                alarm.initialize()
            for index in range(MISSION_SELECTION_BEEP_COUNT):
                alarm.on()
                time.sleep(MISSION_SELECTION_BEEP_ON_S)
                alarm.off()
                if index + 1 < MISSION_SELECTION_BEEP_COUNT:
                    time.sleep(MISSION_SELECTION_BEEP_OFF_S)
            LOG.info("%s acknowledged with three fast alarm pulses", task_name)
        except AlarmGPIOError as exc:
            LOG.warning("%s acknowledgement alarm unavailable: %s", task_name, exc)
        finally:
            self._silence_alarm(alarm)

    def poll(self, now: float) -> None:
        if self.child is not None and self.child.poll() is not None:
            LOG.info("task exited with status %s", self.child.returncode)
            self.child = None
            self.start_idle_reporting()
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
                alarm = _build_alarm(self.car_config)
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
        self._stop_idle_reporting()
        self.child = subprocess.Popen(
            [
                sys.executable,
                str(pending.task_path),
                "--config",
                str(self.config_path),
                "--radar-center-behind-a-cm",
                format(self.radar_center_behind_a_cm, "g"),
            ],
            cwd=self.task_directory,
            start_new_session=True,
        )
        LOG.info("started %s (pid=%d)", pending.task_path.name, self.child.pid)

    def stop(self) -> None:
        self._silence_pending_alarm()
        self._stop_idle_reporting()
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
    parser.add_argument(
        "--config",
        default=None,
        help="vehicle TOML profile (default: CAR_CONFIG env or "
        "configs/cooper_rock5a_l150.toml)",
    )
    parser.add_argument(
        "--port",
        default=None,
        help="override the screen serial device from the profile",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=None,
        help="override the screen baudrate from the profile",
    )
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
        args.task_directory.resolve(),
        args.delay_s,
        args.alarm_duration_s,
        config_path=args.config,
        idle_reporter_factory=IdleFleetReporter,
    )
    if not launcher.car_config.devices.screen.enabled:
        LOG.info(
            "mission screen disabled by vehicle profile "
            "([devices.screen] enabled=false); launcher exits"
        )
        return 0
    screen_port = args.port or launcher.screen_port
    screen_baudrate = args.baudrate or launcher.screen_baudrate
    launcher.start_idle_reporting()
    stopping = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    while not stopping:
        try:
            fd = configure_serial(screen_port, screen_baudrate)
            LOG.info("listening on %s at %d 8N1", screen_port, screen_baudrate)
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
