#!/usr/bin/env python3
"""One-shot GPIO4_B3 active-low alarm test: sound for three seconds, then silence."""

import sys
import time
from pathlib import Path

_COMPONENTS = Path(__file__).resolve().parent.parent / "components"
if str(_COMPONENTS) not in sys.path:
    sys.path.insert(0, str(_COMPONENTS))

from sound_light_alarm import SoundLightAlarm


def main() -> int:
    alarm = SoundLightAlarm().initialize()
    print("[alarm-test] sounding for 3 seconds")
    alarm.on()  # Active-low: drive GPIO4_B3 low.
    try:
        time.sleep(3.0)
    finally:
        alarm.off()  # Restore safe high level even after an interruption.
    print("[alarm-test] alarm silenced")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
