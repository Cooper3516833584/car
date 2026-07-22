#!/usr/bin/env python3
"""Callable HC-14 serial transport compatible with the ground station.

This component owns only the serial link and the ``BB 33`` bridge envelope.
Callers provide and receive complete inner ``AA 22`` protocol frames.  It does
not parse commands, send ACKs, hold an HMAC key, or control any car actuator.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import select
import struct
import threading
import time
from typing import Callable, Final

try:
    import fcntl
    import termios
except ModuleNotFoundError:  # Codec tests are supported on non-Linux hosts.
    fcntl = None
    termios = None


DEFAULT_HC14_PORT: Final[str] = (
    "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
)
DEFAULT_BAUDRATE: Final[int] = 115200
FC_WIRELESS_HEADER: Final[bytes] = b"\xBB\x33"
FC_WIRELESS_MAX_PAYLOAD: Final[int] = 255
READ_CHUNK_SIZE: Final[int] = 4096


class SerialDriverError(RuntimeError):
    """The serial transport is unavailable or a serial operation failed."""


@dataclass(slots=True)
class BridgeCodecStats:
    decoded_frames: int = 0
    discarded_bytes: int = 0
    invalid_lengths: int = 0


class FCWirelessBridgeCodec:
    """Incremental ``BB 33 | length:u8 | payload`` encoder/decoder."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.stats = BridgeCodecStats()

    @staticmethod
    def encode(data: bytes) -> bytes:
        payload = bytes(data)
        if not payload:
            raise ValueError("FC wireless bridge payload must not be empty")
        if len(payload) > FC_WIRELESS_MAX_PAYLOAD:
            raise ValueError("FC wireless bridge payload exceeds 255 bytes")
        return FC_WIRELESS_HEADER + bytes((len(payload),)) + payload

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> list[bytes]:
        if data:
            self._buffer.extend(data)
        payloads: list[bytes] = []
        while True:
            index = self._buffer.find(FC_WIRELESS_HEADER)
            if index < 0:
                keep = 1 if self._buffer[-1:] == FC_WIRELESS_HEADER[:1] else 0
                discarded = len(self._buffer) - keep
                self.stats.discarded_bytes += discarded
                if keep:
                    del self._buffer[:-1]
                else:
                    self._buffer.clear()
                return payloads
            if index:
                self.stats.discarded_bytes += index
                del self._buffer[:index]
            if len(self._buffer) < 3:
                return payloads
            payload_len = self._buffer[2]
            if payload_len == 0:
                self.stats.invalid_lengths += 1
                del self._buffer[0]
                continue
            frame_len = 3 + payload_len
            if len(self._buffer) < frame_len:
                return payloads
            payloads.append(bytes(self._buffer[3:frame_len]))
            self.stats.decoded_frames += 1
            del self._buffer[:frame_len]


def _safe_callback(callback: Callable | None, *args) -> None:
    if callback is not None:
        callback(*args)


class HC14SerialDriver:
    """Threaded HC-14 serial component with automatic reconnect.

    ``on_bytes`` is called with one complete inner payload for each valid bridge
    envelope.  With ``bridge_envelope=False`` it receives raw serial chunks and
    :meth:`write` sends raw bytes; bridge mode should remain enabled when talking
    to the current ground-station implementation.
    """

    def __init__(
        self,
        *,
        on_bytes: Callable[[bytes], None],
        port: str = DEFAULT_HC14_PORT,
        baudrate: int = DEFAULT_BAUDRATE,
        bridge_envelope: bool = True,
        reconnect_seconds: float = 1.0,
        read_timeout_seconds: float = 0.1,
        write_timeout_seconds: float = 0.5,
        on_connected: Callable[[], None] | None = None,
        on_disconnected: Callable[[Exception | None], None] | None = None,
        on_callback_error: Callable[[Exception], None] | None = None,
    ) -> None:
        if not callable(on_bytes):
            raise TypeError("on_bytes must be callable")
        if baudrate not in (9600, 115200):
            raise ValueError("baudrate must be 9600 or 115200")
        if not 0.05 <= reconnect_seconds <= 60.0:
            raise ValueError("reconnect_seconds must be in [0.05, 60]")
        if not 0.01 <= read_timeout_seconds <= 2.0:
            raise ValueError("read_timeout_seconds must be in [0.01, 2]")
        if not 0.05 <= write_timeout_seconds <= 10.0:
            raise ValueError("write_timeout_seconds must be in [0.05, 10]")
        self.port = port
        self.baudrate = baudrate
        self.bridge_envelope = bool(bridge_envelope)
        self.reconnect_seconds = float(reconnect_seconds)
        self.read_timeout_seconds = float(read_timeout_seconds)
        self.write_timeout_seconds = float(write_timeout_seconds)
        self._on_bytes = on_bytes
        self._on_connected = on_connected
        self._on_disconnected = on_disconnected
        self._on_callback_error = on_callback_error
        self._codec = FCWirelessBridgeCodec()
        self._stop = threading.Event()
        self._connected_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._fd: int | None = None
        self._state_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._last_error: Exception | None = None

    @property
    def connected(self) -> bool:
        with self._state_lock:
            return self._fd is not None

    @property
    def last_error(self) -> Exception | None:
        with self._state_lock:
            return self._last_error

    @property
    def codec_stats(self) -> BridgeCodecStats:
        return self._codec.stats

    def start(self) -> "HC14SerialDriver":
        with self._state_lock:
            if self._thread is not None and self._thread.is_alive():
                return self
            self._stop.clear()
            self._connected_event.clear()
            self._last_error = None
            self._thread = threading.Thread(
                target=self._run,
                name="hc14-serial-driver",
                daemon=True,
            )
            self._thread.start()
        return self

    def __enter__(self) -> "HC14SerialDriver":
        return self.start()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def close(self) -> None:
        self._stop.set()
        self._connected_event.clear()
        with self._state_lock:
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(2.0, self.read_timeout_seconds + 0.5))
        with self._state_lock:
            fd = self._fd
            self._fd = None
            self._thread = None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    stop = close

    def wait_connected(self, timeout: float | None = None) -> bool:
        """Wait for the background thread to open the port; return False on timeout."""

        if timeout is not None and timeout < 0.0:
            raise ValueError("timeout must be non-negative or None")
        return self._connected_event.wait(timeout)

    def write(self, data: bytes) -> None:
        """Send one complete inner frame, adding the bridge envelope by default."""

        payload = bytes(data)
        if not payload:
            raise ValueError("serial payload must not be empty")
        outbound = (
            FCWirelessBridgeCodec.encode(payload)
            if self.bridge_envelope
            else payload
        )
        deadline = time.monotonic() + self.write_timeout_seconds
        view = memoryview(outbound)
        with self._write_lock:
            while view:
                with self._state_lock:
                    fd = self._fd
                if fd is None:
                    raise SerialDriverError("HC-14 serial link is not connected")
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise SerialDriverError("HC-14 serial write timed out")
                _, writable, _ = select.select([], [fd], [], remaining)
                if not writable:
                    raise SerialDriverError("HC-14 serial write timed out")
                try:
                    written = os.write(fd, view)
                except OSError as exc:
                    raise SerialDriverError("HC-14 serial write failed") from exc
                if written <= 0:
                    raise SerialDriverError("HC-14 serial write made no progress")
                view = view[written:]

    def _run(self) -> None:
        while not self._stop.is_set():
            disconnect_error: Exception | None = None
            try:
                fd = self._open_serial()
                self._codec.reset()
                with self._state_lock:
                    self._fd = fd
                    self._last_error = None
                self._connected_event.set()
                self._call_lifecycle(self._on_connected)
                self._read_loop(fd)
            except Exception as exc:
                disconnect_error = exc
                with self._state_lock:
                    self._last_error = exc
            finally:
                with self._state_lock:
                    fd = self._fd
                    self._fd = None
                self._connected_event.clear()
                if fd is not None:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
                self._call_lifecycle(self._on_disconnected, disconnect_error)
            if not self._stop.wait(self.reconnect_seconds):
                continue
        with self._state_lock:
            self._thread = None

    def _read_loop(self, fd: int) -> None:
        while not self._stop.is_set():
            readable, _, _ = select.select(
                [fd], [], [], self.read_timeout_seconds
            )
            if not readable:
                continue
            try:
                data = os.read(fd, READ_CHUNK_SIZE)
            except BlockingIOError:
                continue
            if not data:
                raise SerialDriverError("HC-14 serial device disconnected")
            chunks = self._codec.feed(data) if self.bridge_envelope else [data]
            for chunk in chunks:
                try:
                    self._on_bytes(chunk)
                except Exception as exc:
                    self._call_lifecycle(self._on_callback_error, exc)

    def _open_serial(self) -> int:
        if termios is None or fcntl is None:
            raise SerialDriverError("HC-14 serial I/O requires Linux termios")
        speed = {
            9600: termios.B9600,
            115200: termios.B115200,
        }[self.baudrate]
        try:
            fd = os.open(
                self.port,
                os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
            )
        except OSError as exc:
            raise SerialDriverError(f"cannot open HC-14 serial port {self.port}") from exc
        try:
            settings = termios.tcgetattr(fd)
            settings[0] = termios.IGNPAR
            settings[1] = 0
            settings[2] = speed | termios.CS8 | termios.CREAD | termios.CLOCAL
            settings[3] = 0
            settings[4] = speed
            settings[5] = speed
            settings[6][termios.VMIN] = 0
            settings[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, settings)
            termios.tcflush(fd, termios.TCIOFLUSH)
            clear_lines = termios.TIOCM_DTR | termios.TIOCM_RTS
            fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", clear_lines))
            return fd
        except BaseException:
            os.close(fd)
            raise

    def _call_lifecycle(self, callback: Callable | None, *args) -> None:
        try:
            _safe_callback(callback, *args)
        except Exception as exc:
            if callback is not self._on_callback_error:
                try:
                    _safe_callback(self._on_callback_error, exc)
                except Exception:
                    pass


# Clear, descriptive alias for application code that does not need HC-14 naming.
SerialCommunicationDriver = HC14SerialDriver
