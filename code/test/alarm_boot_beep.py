#!/usr/bin/env python3
"""Boot-time active-low alarm self-test: sound for three seconds, then stop."""

import sys
import time
from pathlib import Path

_COMPONENTS = Path(__file__).resolve().parent.parent / "components"
if str(_COMPONENTS) not in sys.path:
    sys.path.insert(0, str(_COMPONENTS))

from sound_light_alarm import SoundLightAlarm

BOOT_BEEP_SECONDS = 3.0


def main() -> int:
    alarm = SoundLightAlarm().initialize()
    print(f"[alarm-startup] GPIO initialized; sounding for {BOOT_BEEP_SECONDS:g} s")
    alarm.on()
    try:
        time.sleep(BOOT_BEEP_SECONDS)
    finally:
        alarm.off()
    print("[alarm-startup] done; alarm silenced")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
