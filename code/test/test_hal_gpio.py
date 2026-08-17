"""Hardware-free tests for the Linux sysfs GPIO HAL using a fake sysfs tree.

Never touches /sys/class/gpio on the host.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import unittest
import uuid

from hal.gpio import (
    GPIOBackendError,
    LinuxSysfsBankGPIOOutput,
    resolve_gpio_number,
)


class LinuxSysfsBankGPIOOutputTests(unittest.TestCase):
    def setUp(self) -> None:
        self.root = (
            Path(__file__).resolve().parent / f"_hal_tmp_{uuid.uuid4().hex}"
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
        (gpio / "value").write_text("1\n", encoding="ascii")

    def test_resolve_gpio_number_uses_label_and_base(self) -> None:
        self.assertEqual(
            resolve_gpio_number(
                self.root, bank_label="gpio4", bank_offset=11
            ),
            139,
        )

    def test_missing_bank_raises(self) -> None:
        with self.assertRaisesRegex(GPIOBackendError, "gpio9"):
            resolve_gpio_number(
                self.root, bank_label="gpio9", bank_offset=0
            )

    def test_initialize_selects_safe_high(self) -> None:
        output = LinuxSysfsBankGPIOOutput(
            sysfs_root=self.root,
            bank_label="gpio4",
            line_offset=11,
            active_low=True,
        ).initialize()

        self.assertEqual(
            (self.root / "gpio139/direction").read_text().strip(), "high"
        )
        self.assertFalse(output.is_active)

    def test_active_low_polarity_controls_raw_value(self) -> None:
        output = LinuxSysfsBankGPIOOutput(
            sysfs_root=self.root,
            bank_label="gpio4",
            line_offset=11,
            active_low=True,
        ).initialize()

        output.set_active(True)
        self.assertTrue(output.is_active)
        self.assertEqual(
            (self.root / "gpio139/value").read_text().strip(), "0"
        )

        output.set_active(False)
        self.assertFalse(output.is_active)
        self.assertEqual(
            (self.root / "gpio139/value").read_text().strip(), "1"
        )

    def test_high_active_polarity_inverts_raw_value(self) -> None:
        output = LinuxSysfsBankGPIOOutput(
            sysfs_root=self.root,
            bank_label="gpio4",
            line_offset=11,
            active_low=False,
        ).initialize()

        output.set_active(True)
        self.assertTrue(output.is_active)
        self.assertEqual(
            (self.root / "gpio139/value").read_text().strip(), "1"
        )

        output.set_active(False)
        self.assertEqual(
            (self.root / "gpio139/value").read_text().strip(), "0"
        )

    def test_uninitialized_output_is_rejected(self) -> None:
        output = LinuxSysfsBankGPIOOutput(
            sysfs_root=self.root,
            bank_label="gpio4",
            line_offset=11,
            active_low=True,
            gpio_number=140,
        )
        with self.assertRaisesRegex(GPIOBackendError, "initialize"):
            output.set_active(True)

    def test_explicit_gpio_number_skips_bank_resolution(self) -> None:
        output = LinuxSysfsBankGPIOOutput(
            sysfs_root=self.root,
            bank_label="gpio4",
            line_offset=11,
            active_low=True,
            gpio_number=139,
        )
        self.assertEqual(output.gpio_number, 139)


if __name__ == "__main__":
    unittest.main()
