#!/usr/bin/env python3
"""Boot-time alarm self-test: sound for 3 seconds then stop.

Uses the shared ``SoundLightAlarm`` component so project code can also call
``alarm_on()`` / ``alarm_off()`` directly.
"""

import sys
from pathlib import Path

# Allow this test script to import from the sibling components directory.
_COMPONENTS = Path(__file__).resolve().parent.parent / "components"
if str(_COMPONENTS) not in sys.path:
    sys.path.insert(0, str(_COMPONENTS))

from sound_light_alarm import SoundLightAlarm

BOOT_BEEP_SECONDS: float = 3.0


def main() -> int:
    alarm = SoundLightAlarm()

    # initialize() atomically selects output-high before any direction change,
    # so the alarm is silent while the GPIO is being set up.
    alarm.initialize()

    print("[alarm-startup] GPIO initialized — sounding for", BOOT_BEEP_SECONDS, "s")
    alarm.on()                       # drive low  -> alarm ON
    import time
    time.sleep(BOOT_BEEP_SECONDS)
    alarm.off()                      # drive high -> alarm OFF
    print("[alarm-startup] done — alarm silenced")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
