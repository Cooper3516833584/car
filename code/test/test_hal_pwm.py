"""Hardware-free tests for the Linux sysfs PWM HAL using a fake sysfs tree.

Never touches /sys/class/pwm on the host.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import unittest
import unittest.mock
import uuid

from hal.pwm import LinuxSysfsPWMOutput, PWMBackendError


class LinuxSysfsPWMOutputTests(unittest.TestCase):
    def setUp(self) -> None:
        self.root = (
            Path(__file__).resolve().parent / f"_hal_tmp_{uuid.uuid4().hex}"
        )
        self.root.mkdir()
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)

    def _make_chip(self, name: str = "pwmchip0") -> Path:
        chip = self.root / name
        chip.mkdir()
        (chip / "export").write_text("", encoding="ascii")
        return chip

    def _make_pwm(self, chip: Path) -> Path:
        pwm = chip / "pwm0"
        pwm.mkdir()
        (pwm / "enable").write_text("0\n", encoding="ascii")
        (pwm / "period").write_text("0\n", encoding="ascii")
        (pwm / "polarity").write_text("normal\n", encoding="ascii")
        (pwm / "duty_cycle").write_text("0\n", encoding="ascii")
        return pwm

    def test_start_configures_period_and_polarity_and_export(self) -> None:
        chip = self._make_chip()
        pwm = self._make_pwm(chip)
        (chip / "export").write_text("", encoding="ascii")

        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="pwmchip0",
            channel=0,
            period_ns=20_000_000,
            polarity="normal",
        )
        output.start()

        self.assertEqual((pwm / "period").read_text().strip(), "20000000")
        self.assertEqual((pwm / "polarity").read_text().strip(), "normal")
        self.assertTrue(output.is_running)

    def test_set_pulse_us_writes_ns_duty_and_enables(self) -> None:
        chip = self._make_chip()
        pwm = self._make_pwm(chip)
        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="pwmchip0",
            channel=0,
            period_ns=20_000_000,
            polarity="normal",
        )
        output.start()

        output.set_pulse_us(1580)

        self.assertEqual((pwm / "duty_cycle").read_text().strip(), "1580000")
        self.assertEqual((pwm / "enable").read_text().strip(), "1")

    def test_disable_writes_enable_zero(self) -> None:
        chip = self._make_chip()
        pwm = self._make_pwm(chip)
        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="pwmchip0",
            channel=0,
        )
        output.start()
        output.set_pulse_us(1500)
        output.disable()

        self.assertEqual((pwm / "enable").read_text().strip(), "0")

    def test_export_creates_missing_pwm_channel(self) -> None:
        chip = self._make_chip()
        (chip / "export").write_text("", encoding="ascii")
        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="pwmchip0",
            channel=0,
        )
        with unittest.mock.patch("hal.pwm.time.sleep"):
            with self.assertRaisesRegex(PWMBackendError, "export"):
                output.start()

        # The export request was written even though the fake sysfs never
        # creates the channel directory.
        self.assertEqual((chip / "export").read_text().strip(), "0")

    def test_operations_before_start_raise(self) -> None:
        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="pwmchip0",
            channel=0,
        )
        with self.assertRaises(PWMBackendError):
            output.set_pulse_us(1500)
        with self.assertRaises(PWMBackendError):
            output.disable()

    def test_missing_chip_raises_clear_error(self) -> None:
        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="does-not-exist",
            channel=0,
        )
        with self.assertRaisesRegex(PWMBackendError, "does-not-exist"):
            output.start()


if __name__ == "__main__":
    unittest.main()
