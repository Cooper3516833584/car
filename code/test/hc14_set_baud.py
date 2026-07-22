#!/usr/bin/env python3
"""Set one HC-14 UART baud rate over SSH and verify it by readback."""

from __future__ import annotations

import argparse
import os
import sys

import paramiko


REMOTE_SET_BAUD = r'''
import fcntl, os, struct, sys, termios, time

port, old_baud, new_baud = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])

def open_port(baud):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, "B" + str(baud))
    attrs[0] = termios.IGNPAR; attrs[1] = 0
    attrs[2] = speed | termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0; attrs[4] = speed; attrs[5] = speed
    attrs[6][termios.VMIN] = 0; attrs[6][termios.VTIME] = 2
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS))
    fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_RTS))
    time.sleep(0.5)
    return fd

def exchange(fd, command, wait=1.0):
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, command)
    reply = b""
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
        reply += os.read(fd, 256)
    return reply

fd = open_port(old_baud)
try:
    command = ("AT+B%d" % new_baud).encode("ascii")
    reply = exchange(fd, command, 1.5)
    print(command.decode() + " => " + reply.decode("ascii", "replace").replace("\r", "\\r").replace("\n", "\\n"))
    if ("OK+B:%d" % new_baud).encode("ascii") not in reply:
        raise SystemExit("HC-14 did not acknowledge the requested baud rate")
finally:
    fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS))
    os.close(fd)

time.sleep(0.6)
fd = open_port(new_baud)
try:
    reply = exchange(fd, b"AT+RX", 1.5)
    rendered = reply.decode("ascii", "replace").replace("\r", "\\r").replace("\n", "\\n")
    print("VERIFY AT+RX => " + rendered)
    expected = ("OK+B:%d" % new_baud).encode("ascii")
    if expected not in reply:
        raise SystemExit("HC-14 baud-rate verification failed")
finally:
    fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS))
    os.close(fd)
'''


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--password-env", required=True)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--old-baud", type=int, required=True)
    parser.add_argument("--new-baud", type=int, required=True)
    args = parser.parse_args()
    password = os.environ.get(args.password_env)
    if not password:
        parser.error(f"{args.password_env} is not set")

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(args.host, username=args.user, password=password, timeout=10, banner_timeout=10)
        command = "sudo -S -p '' python3 -u - {} {} {}".format(
            args.port, args.old_baud, args.new_baud
        )
        stdin, stdout, stderr = client.exec_command(command, timeout=20)
        stdin.write(password + "\n" + REMOTE_SET_BAUD)
        stdin.channel.shutdown_write()
        out, err = stdout.read(), stderr.read()
        sys.stdout.write(out.decode("utf-8", "replace") if isinstance(out, bytes) else out)
        sys.stderr.write(err.decode("utf-8", "replace") if isinstance(err, bytes) else err)
        return stdout.channel.recv_exit_status()
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
