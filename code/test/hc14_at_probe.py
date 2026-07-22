#!/usr/bin/env python3
"""Read-only HC-14 AT probe over SSH using Linux standard-library serial I/O."""

from __future__ import annotations

import argparse
import os
import sys

import paramiko


REMOTE_PROBE = r'''
import fcntl, os, struct, sys, termios, time
port, baud = sys.argv[1], int(sys.argv[2])
dtr, rts = bool(int(sys.argv[3])), bool(int(sys.argv[4]))
fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
attrs = termios.tcgetattr(fd)
attrs[0] = termios.IGNPAR; attrs[1] = 0
attrs[2] = getattr(termios, "B" + str(baud)) | termios.CS8 | termios.CREAD | termios.CLOCAL
attrs[3] = 0; attrs[4] = getattr(termios, "B" + str(baud)); attrs[5] = getattr(termios, "B" + str(baud))
attrs[6][termios.VMIN] = 0; attrs[6][termios.VTIME] = 2
termios.tcflush(fd, termios.TCIOFLUSH); termios.tcsetattr(fd, termios.TCSANOW, attrs)
all_lines = termios.TIOCM_DTR | termios.TIOCM_RTS
fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", all_lines))
set_lines = (termios.TIOCM_DTR if dtr else 0) | (termios.TIOCM_RTS if rts else 0)
if set_lines:
    fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", set_lines))
time.sleep(0.5)
print("CONTROL DTR=%s RTS=%s BAUD=%d" % (dtr, rts, baud))
try:
    # HC-14 commands are sent without CR/LF; line endings are parsed as
    # additional invalid commands and produce ORDER ERROR.
    for command in (b"AT", b"AT+B?", b"AT+C?", b"AT+S?", b"AT+P?", b"AT+RX"):
        os.write(fd, command); time.sleep(0.35)
        reply = b""
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            reply += os.read(fd, 256)
        print(command.decode().strip() + " => " + reply.decode("ascii", "replace").replace("\r", "\\r").replace("\n", "\\n"))
finally:
    os.close(fd)
'''


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--password-env", required=True)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, required=True, choices=(9600, 115200))
    parser.add_argument("--dtr", action="store_true")
    parser.add_argument("--rts", action="store_true")
    args = parser.parse_args()
    password = os.environ.get(args.password_env)
    if not password:
        parser.error(f"{args.password_env} is not set")

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(args.host, username=args.user, password=password, timeout=10, banner_timeout=10)
        command = "sudo -S -p '' python3 -u - {} {} {} {}".format(
            args.port, args.baud, int(args.dtr), int(args.rts)
        )
        stdin, stdout, stderr = client.exec_command(command, timeout=15)
        stdin.write(password + "\n" + REMOTE_PROBE)
        stdin.channel.shutdown_write()
        out, err = stdout.read(), stderr.read()
        sys.stdout.write(out.decode("utf-8", "replace") if isinstance(out, bytes) else out)
        sys.stderr.write(err.decode("utf-8", "replace") if isinstance(err, bytes) else err)
        return stdout.channel.recv_exit_status()
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
