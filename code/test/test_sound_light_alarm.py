"""Hardware-free tests for the active-low sound/light alarm component."""

from __future__ import annotations

from pathlib import Path
import shutil
import unittest
import uuid

from components.sound_light_alarm import AlarmGPIOError, SoundLightAlarm, resolve_gpio_number


class SoundLightAlarmTests(unittest.TestCase):
    def setUp(self) -> None:
        # Use Path.mkdir (not tempfile.mkdtemp): the Windows file sandbox
        # denies creating subdirectories inside mkdtemp-created directories.
        self.root = (
            Path(__file__).resolve().parent / f"_alarm_tmp_{uuid.uuid4().hex}"
        )
        self.root.mkdir()
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)
        chip = self.root / "gpiochip128"
        chip.mkdir()
        (chip / "label").write_text("gpio4\n", encoding="ascii")
        (chip / "base").write_text("128\n", encoding="ascii")
        (chip / "ngpio").write_text("32\n", encoding="ascii")
        gpio = self.root / "gpio139"
        gpio.mkdir()
        (gpio / "direction").write_text("in\n", encoding="ascii")
        (gpio / "value").write_text("0\n", encoding="ascii")

    def test_resolves_gpio4_b3_to_global_139(self) -> None:
        self.assertEqual(resolve_gpio_number(self.root), 139)

    def test_initialize_drives_safe_high_then_active_low_controls_alarm(self) -> None:
        alarm = SoundLightAlarm(sysfs_gpio_root=self.root).initialize()
        self.assertEqual((self.root / "gpio139/direction").read_text().strip(), "high")

        alarm.off()
        self.assertFalse(alarm.is_active)
        self.assertEqual((self.root / "gpio139/value").read_text().strip(), "1")

        alarm.on()
        self.assertTrue(alarm.is_active)
        self.assertEqual((self.root / "gpio139/value").read_text().strip(), "0")

    def test_set_active_uses_active_low_polarity(self) -> None:
        alarm = SoundLightAlarm(sysfs_gpio_root=self.root).initialize()
        alarm.set_active(True)
        self.assertTrue(alarm.is_active)
        alarm.set_active(False)
        self.assertFalse(alarm.is_active)

    def test_uninitialized_output_is_rejected(self) -> None:
        alarm = SoundLightAlarm(gpio_number=140, sysfs_gpio_root=self.root)
        with self.assertRaises(AlarmGPIOError):
            alarm.off()


if __name__ == "__main__":
    unittest.main()
