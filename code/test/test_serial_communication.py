"""Hardware-free tests for the HC-14 bridge codec and component validation."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.serial_communication import (  # noqa: E402
    FCWirelessBridgeCodec,
    HC14SerialDriver,
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
    def test_default_component_is_bridge_enabled(self) -> None:
        driver = HC14SerialDriver(on_bytes=lambda data: None)
        self.assertTrue(driver.bridge_envelope)
        self.assertEqual(driver.baudrate, 115200)
        self.assertFalse(driver.connected)
        self.assertFalse(driver.wait_connected(0.0))

    def test_callback_is_required(self) -> None:
        with self.assertRaises(TypeError):
            HC14SerialDriver(on_bytes=None)  # type: ignore[arg-type]


if __name__ == "__main__":
    unittest.main()
