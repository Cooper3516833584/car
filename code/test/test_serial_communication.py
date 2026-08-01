"""Hardware-free tests for the HC-14 bridge codec and component validation."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.serial_communication import (  # noqa: E402
    FCWirelessBridgeCodec,
    HC14SerialDriver,
    SerialDriverError,
)


class FCWirelessBridgeCodecTests(unittest.TestCase):
    def test_ground_station_compatible_encoding(self) -> None:
        self.assertEqual(
            FCWirelessBridgeCodec.encode(b"\xAA\x22\x01"),
            b"\xBB\x33\x03\xAA\x22\x01",
        )

    def test_fragmented_frame(self) -> None:
        codec = FCWirelessBridgeCodec()
        frame = FCWirelessBridgeCodec.encode(b"\xAA\x22payload")
        self.assertEqual(codec.feed(frame[:1]), [])
        self.assertEqual(codec.feed(frame[1:4]), [])
        self.assertEqual(codec.feed(frame[4:]), [b"\xAA\x22payload"])
        self.assertEqual(codec.stats.decoded_frames, 1)

    def test_noise_resynchronization_and_multiple_frames(self) -> None:
        codec = FCWirelessBridgeCodec()
        stream = (
            b"noise"
            + FCWirelessBridgeCodec.encode(b"one")
            + FCWirelessBridgeCodec.encode(b"two")
        )
        self.assertEqual(codec.feed(stream), [b"one", b"two"])
        self.assertEqual(codec.stats.discarded_bytes, 5)

    def test_invalid_zero_length_recovers(self) -> None:
        codec = FCWirelessBridgeCodec()
        data = b"\xBB\x33\x00" + FCWirelessBridgeCodec.encode(b"ok")
        self.assertEqual(codec.feed(data), [b"ok"])
        self.assertEqual(codec.stats.invalid_lengths, 1)

    def test_payload_limits(self) -> None:
        with self.assertRaises(ValueError):
            FCWirelessBridgeCodec.encode(b"")
        with self.assertRaises(ValueError):
            FCWirelessBridgeCodec.encode(bytes(256))


class HC14SerialDriverValidationTests(unittest.TestCase):
    @staticmethod
    def _fake_termios():
        return SimpleNamespace(
            B9600=9600,
            B115200=115200,
            IGNPAR=1,
            CS8=2,
            CREAD=4,
            CLOCAL=8,
            VMIN=6,
            VTIME=5,
            TCSANOW=0,
            TCIOFLUSH=2,
            TIOCM_DTR=2,
            TIOCM_RTS=4,
            TIOCMBIC=8,
            tcgetattr=mock.Mock(return_value=[0, 0, 0, 0, 0, 0, [0] * 32]),
            tcsetattr=mock.Mock(),
            tcflush=mock.Mock(),
        )

    def test_default_component_is_bridge_enabled(self) -> None:
        driver = HC14SerialDriver(on_bytes=lambda data: None)
        self.assertTrue(driver.bridge_envelope)
        self.assertEqual(driver.baudrate, 115200)
        self.assertFalse(driver.connected)
        self.assertFalse(driver.wait_connected(0.0))

    def test_callback_is_required(self) -> None:
        with self.assertRaises(TypeError):
            HC14SerialDriver(on_bytes=None)  # type: ignore[arg-type]

    def test_open_serial_acquires_process_exclusive_lock(self) -> None:
        fake_fcntl = SimpleNamespace(
            LOCK_EX=1,
            LOCK_NB=2,
            flock=mock.Mock(),
            ioctl=mock.Mock(),
        )
        fake_termios = self._fake_termios()
        driver = HC14SerialDriver(on_bytes=lambda data: None)

        with mock.patch.multiple(
                "components.serial_communication.os",
                O_RDWR=2,
                O_NOCTTY=0,
                O_NONBLOCK=0,
                create=True,
        ), mock.patch("components.serial_communication.fcntl", fake_fcntl), \
                mock.patch("components.serial_communication.termios", fake_termios), \
                mock.patch("components.serial_communication.os.open", return_value=41), \
                mock.patch("components.serial_communication.os.close") as close:
            self.assertEqual(41, driver._open_serial())

        fake_fcntl.flock.assert_called_once_with(41, 3)
        fake_termios.tcgetattr.assert_called_once_with(41)
        close.assert_not_called()

    def test_open_serial_rejects_second_process_and_closes_fd(self) -> None:
        fake_fcntl = SimpleNamespace(
            LOCK_EX=1,
            LOCK_NB=2,
            flock=mock.Mock(side_effect=BlockingIOError("held")),
            ioctl=mock.Mock(),
        )
        fake_termios = self._fake_termios()
        driver = HC14SerialDriver(on_bytes=lambda data: None)

        with mock.patch.multiple(
                "components.serial_communication.os",
                O_RDWR=2,
                O_NOCTTY=0,
                O_NONBLOCK=0,
                create=True,
        ), mock.patch("components.serial_communication.fcntl", fake_fcntl), \
                mock.patch("components.serial_communication.termios", fake_termios), \
                mock.patch("components.serial_communication.os.open", return_value=42), \
                mock.patch("components.serial_communication.os.close") as close:
            with self.assertRaisesRegex(SerialDriverError, "already in use"):
                driver._open_serial()

        close.assert_called_once_with(42)
        fake_termios.tcgetattr.assert_not_called()


if __name__ == "__main__":
    unittest.main()
