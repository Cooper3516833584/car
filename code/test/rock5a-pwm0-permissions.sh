#!/bin/sh
set -eu

CHIP=/sys/class/pwm/pwmchip0
PWM="$CHIP/pwm0"

if [ ! -d "$PWM" ]; then
    printf '0\n' > "$CHIP/export"
    attempts=0
    while [ ! -d "$PWM" ] && [ "$attempts" -lt 20 ]; do
        sleep 0.05
        attempts=$((attempts + 1))
    done
fi

if [ ! -d "$PWM" ]; then
    echo "pwm0 export failed" >&2
    exit 1
fi

chgrp pwm "$CHIP/export" "$CHIP/unexport" \
    "$PWM/enable" "$PWM/period" "$PWM/duty_cycle" "$PWM/polarity"
chmod g+rw "$CHIP/export" "$CHIP/unexport" \
    "$PWM/enable" "$PWM/period" "$PWM/duty_cycle" "$PWM/polarity"
