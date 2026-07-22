#!/usr/bin/env python3
"""Calibrated, low-travel Ackermann steering-servo test on ROCK 5A Pin 23."""

from __future__ import annotations

import argparse
import time
from pathlib import Path


PWM_PERIOD_NS = 20_000_000  # 50 Hz
SERVO_MIN_US = 800
SERVO_MAX_US = 2200
STEERING_DIRECTION_SIGN = -1.0
STEERING_MIN_RAD = -0.32
STEERING_MAX_RAD = 0.49
FACTORY_CENTER_US = 1501
STEERING_CENTER_US = 1580


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def steering_angle_to_pulse_us(angle_rad: float) -> int:
    """WHEELTEC curve with this car's confirmed reversed servo/linkage sign."""
    angle = clamp(angle_rad, STEERING_MIN_RAD, STEERING_MAX_RAD)
    calibration_angle = STEERING_DIRECTION_SIGN * angle
    servo_angle = (
        -0.628 * calibration_angle**3
        + 1.269 * calibration_angle**2
        - 1.772 * calibration_angle
        + 1.573
    )
    factory_pulse = 1500 + (servo_angle - 1.572) * 640.62
    pulse = factory_pulse + (STEERING_CENTER_US - FACTORY_CENTER_US)
    return round(clamp(pulse, SERVO_MIN_US, SERVO_MAX_US))


def pwm0_chip() -> Path:
    for chip in Path("/sys/class/pwm").glob("pwmchip*"):
        if "fd8b0000.pwm" in str(chip.resolve()):
            return chip
    raise RuntimeError(
        "PWM0 is unavailable. Enable rk3588-pwm0-m2 with rsetup and reboot first."
    )


def write(path: Path, value: int | str) -> None:
    path.write_text(f"{value}\n")


def setup_pwm() -> Path:
    chip = pwm0_chip()
    pwm = chip / "pwm0"
    if not pwm.exists():
        write(chip / "export", 0)
        for _ in range(20):
            if pwm.exists():
                break
            time.sleep(0.05)
    if not pwm.exists():
        raise RuntimeError("PWM0 export did not create pwm0")
    # A freshly exported Rockchip PWM has period=0; writing enable=0 at that
    # point is rejected by this kernel.  Disable only if it was already active.
    if (pwm / "enable").read_text().strip() == "1":
        write(pwm / "enable", 0)
    write(pwm / "period", PWM_PERIOD_NS)
    write(pwm / "polarity", "normal")
    return pwm


def set_angle(pwm: Path, angle_rad: float) -> int:
    pulse_us = steering_angle_to_pulse_us(angle_rad)
    write(pwm / "duty_cycle", pulse_us * 1000)
    write(pwm / "enable", 1)
    print(f"theta={angle_rad:+.3f} rad -> {pulse_us} us")
    return pulse_us


def set_pulse(pwm: Path, pulse_us: int) -> None:
    """Set an explicit pulse for mechanical centre calibration only."""

    if not SERVO_MIN_US <= pulse_us <= SERVO_MAX_US:
        raise ValueError(f"pulse must be in {SERVO_MIN_US}..{SERVO_MAX_US} us")
    write(pwm / "duty_cycle", pulse_us * 1000)
    write(pwm / "enable", 1)
    print(f"direct pulse={pulse_us} us")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sweep", action="store_true", help="center, small negative turn, small positive turn, center"
    )
    parser.add_argument("--angle-rad", type=float, help="set one front-wheel steering angle")
    parser.add_argument("--pulse-us", type=int, help="set one raw pulse for centre calibration")
    parser.add_argument("--hold-seconds", type=float, default=0.8)
    args = parser.parse_args()
    if not 0.2 <= args.hold_seconds <= 3.0:
        parser.error("--hold-seconds must be from 0.2 to 3")
    selected_modes = int(args.sweep) + int(args.angle_rad is not None) + int(args.pulse_us is not None)
    if selected_modes != 1:
        parser.error("choose exactly one of --sweep, --angle-rad, or --pulse-us")

    pwm = setup_pwm()
    if args.sweep:
        # About +/- 7 degrees: deliberately below the source-model steering limits.
        for angle in (0.0, -0.12, 0.12, 0.0):
            set_angle(pwm, angle)
            time.sleep(args.hold_seconds)
    elif args.angle_rad is not None:
        set_angle(pwm, args.angle_rad)
    else:
        try:
            set_pulse(pwm, args.pulse_us)
        except ValueError as exc:
            parser.error(str(exc))

    # Leave PWM enabled at the last requested angle so the servo holds position.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
