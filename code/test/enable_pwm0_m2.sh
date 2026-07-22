#!/usr/bin/env bash
# Enables the official ROCK 5A overlay for PWM0_M2 (physical header Pin 23).
set -euo pipefail

DEBUG=false
source /usr/lib/rsetup/cli/main.sh
source /usr/lib/rsetup/cli/u-boot-menu.sh
enable_overlays rk3588-pwm0-m2.dtbo
