#!/usr/bin/env python3
"""Direct raw-PWM front-servo calibration tool for the ROCK 5A.

Edit ``PULSE_US`` below and click Run.  This file controls only PWM0 on
physical Pin 23 and never opens /dev/ttyACM0, so it cannot command the rear
motors.  The pulse remains enabled after exit so the steering holds position.
"""

from __future__ import annotations

from pathlib import Path
import time


# Edit this value, save the file, then click Run.
PULSE_US = 1580

PWM_PERIOD_NS = 20_000_000
PULSE_MIN_US = 800
PULSE_MAX_US = 2200


def write(path: Path, value: int | str) -> None:
    path.write_text(f"{value}\n", encoding="ascii")


def find_pwm0_chip() -> Path:
    for chip in Path("/sys/class/pwm").glob("pwmchip*"):
        if "fd8b0000.pwm" in str(chip.resolve()):
            return chip
    raise RuntimeError("PWM0 unavailable; enable rk3588-pwm0-m2 and reboot")


def main() -> int:
    if not PULSE_MIN_US <= PULSE_US <= PULSE_MAX_US:
        raise ValueError(f"PULSE_US must be in {PULSE_MIN_US}..{PULSE_MAX_US}")
    chip = find_pwm0_chip()
    pwm = chip / "pwm0"
    if not pwm.exists():
        write(chip / "export", 0)
        for _ in range(20):
            if pwm.exists():
                break
            time.sleep(0.05)
    if not pwm.exists():
        raise RuntimeError("PWM0 export did not create pwm0")
    enable = pwm / "enable"
    if enable.read_text(encoding="ascii").strip() == "1":
        write(enable, 0)
    write(pwm / "period", PWM_PERIOD_NS)
    write(pwm / "polarity", "normal")
    write(pwm / "duty_cycle", PULSE_US * 1000)
    write(enable, 1)
    print(f"front steering only: PULSE_US={PULSE_US} us")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
