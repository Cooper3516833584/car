#!/usr/bin/env python3
"""One-shot deploy: copy alarm boot-beep script & systemd service to ROCK 5A.

Usage (from Windows PowerShell, after the board is reachable at 192.168.31.224):

    python code/test/deploy_alarm_boot_beep.py
"""

import os
import sys
import time
from pathlib import Path

BOARD_HOST = os.environ.get("ROCK5A_HOST", "192.168.31.224")
BOARD_USER = os.environ.get("ROCK5A_USER", "radxa")
BOARD_PASS = os.environ.get("ROCK5A_PASS", "11223344")

_PROJECT = Path(__file__).resolve().parent.parent.parent  # car/

FILES_TO_COPY = [
    # (local_rel_path, remote_abs_path)
    ("code/components/sound_light_alarm.py", "/home/radxa/car/code/components/sound_light_alarm.py"),
    ("code/test/alarm_boot_beep.py",        "/home/radxa/car/code/test/alarm_boot_beep.py"),
    ("code/test/alarm-boot-beep.service",   "/home/radxa/car/code/test/alarm-boot-beep.service"),
]

SYSTEMD_COMMANDS = [
    "sudo cp /home/radxa/car/code/test/alarm-boot-beep.service /etc/systemd/system/",
    "sudo systemctl daemon-reload",
    "sudo systemctl enable alarm-boot-beep.service",
    "echo '>>> Service enabled. Reboot to test, or run: sudo systemctl start alarm-boot-beep'",
]


def main() -> int:
    import paramiko

    print(f"Connecting to {BOARD_USER}@{BOARD_HOST} ...")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        c.connect(BOARD_HOST, username=BOARD_USER, password=BOARD_PASS, timeout=15)
    except Exception as exc:
        print(f"FAILED to connect: {exc}", file=sys.stderr)
        return 1

    sftp = c.open_sftp()

    for local_rel, remote_abs in FILES_TO_COPY:
        local = _PROJECT / local_rel
        if not local.is_file():
            print(f"  SKIP  {local_rel}  (not found)", file=sys.stderr)
            continue
        print(f"  PUT   {local_rel}  ->  {remote_abs}")
        sftp.put(str(local), remote_abs, confirm=True)

    sftp.close()

    print("\nEnabling systemd service ...")
    for cmd in SYSTEMD_COMMANDS:
        print(f"  $ {cmd}")
        _, stdout, stderr = c.exec_command(cmd, timeout=10)
        out = stdout.read().decode(errors="replace").strip()
        err = stderr.read().decode(errors="replace").strip()
        if out:
            print(f"    {out}")
        if err:
            print(f"    [stderr] {err}", file=sys.stderr)

    c.close()
    print("\nDone. You can now test with:")
    print("  sudo systemctl start alarm-boot-beep")
    print("Or reboot the board to see the 3-second self-test at boot.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
