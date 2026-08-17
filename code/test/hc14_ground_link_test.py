#!/usr/bin/env python3
"""Low-rate bidirectional transparent-serial test: ground station <-> car.

The test sends only ASCII probe lines.  Both serial ports are opened at 115200
8N1 with DTR and RTS explicitly deasserted, as required by the HC-14 setup.
Real hosts/passwords are read from the environment; nothing machine-private
is committed.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import uuid

import paramiko


def _env_host(name: str, default: str) -> str:
    return os.environ.get(name, default)


def _env_user(name: str, default: str) -> str:
    return os.environ.get(name, default)


def _env_password(name: str) -> str:
    password = os.environ.get(name, "")
    if not password:
        raise SystemExit(f"{name} is not set")
    return password


def targets() -> tuple[tuple[str, str, str, str], tuple[str, str, str, str]]:
    """(host, user, device, password) for ground station and car."""
    ground = (
        _env_host("GROUND_STATION_HOST", "ground-station.local"),
        _env_user("GROUND_STATION_USER", "user"),
        "/dev/ttyUSB0",
        _env_password("GROUND_STATION_PASSWORD"),
    )
    car = (
        _env_host("ROCK5A_HOST", "car.local"),
        _env_user("ROCK5A_USER", "user"),
        "/dev/ttyUSB0",
        _env_password("ROCK5A_PASSWORD"),
    )
    return ground, car


def connect(host: str, user: str, password: str) -> paramiko.SSHClient:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username=user, password=password, timeout=10, banner_timeout=10)
    return client


def receiver_code(port: str, count: int, timeout: float) -> str:
    return f"""
import fcntl, os, struct, termios, time
fd = os.open({port!r}, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
attrs = termios.tcgetattr(fd)
attrs[0] = termios.IGNPAR; attrs[1] = 0
attrs[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
attrs[3] = 0; attrs[4] = termios.B115200; attrs[5] = termios.B115200
attrs[6][termios.VMIN] = 0; attrs[6][termios.VTIME] = 2
termios.tcflush(fd, termios.TCIOFLUSH); termios.tcsetattr(fd, termios.TCSANOW, attrs)
fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack('I', termios.TIOCM_DTR | termios.TIOCM_RTS))
print('READY', flush=True)
deadline = time.monotonic() + {timeout}; buf = b''; lines = []
try:
    while time.monotonic() < deadline and len(lines) < {count}:
        data = os.read(fd, 256)
        if not data: continue
        buf += data
        while b'\\n' in buf:
            line, buf = buf.split(b'\\n', 1)
            text = line.decode('ascii', 'replace').strip()
            if text:
                lines.append(text); print('RX ' + text, flush=True)
finally:
    os.close(fd)
print('COUNT ' + str(len(lines)), flush=True)
"""


def sender_code(port: str, payloads: list[str]) -> str:
    return f"""
import fcntl, os, struct, termios, time
payloads = {payloads!r}
fd = os.open({port!r}, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
attrs = termios.tcgetattr(fd)
attrs[0] = termios.IGNPAR; attrs[1] = 0
attrs[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
attrs[3] = 0; attrs[4] = termios.B115200; attrs[5] = termios.B115200
attrs[6][termios.VMIN] = 0; attrs[6][termios.VTIME] = 2
termios.tcflush(fd, termios.TCIOFLUSH); termios.tcsetattr(fd, termios.TCSANOW, attrs)
fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack('I', termios.TIOCM_DTR | termios.TIOCM_RTS))
try:
    for payload in payloads:
        os.write(fd, (payload + '\\n').encode('ascii'))
        print('TX ' + payload, flush=True); time.sleep(0.5)
finally:
    os.close(fd)
"""


def start_sudo_python(client: paramiko.SSHClient, password: str, code: str):
    stdin, stdout, stderr = client.exec_command("sudo -S -p '' python3 -u -", timeout=20)
    stdin.write(password + "\n" + code)
    stdin.channel.shutdown_write()
    return stdout, stderr


def text_of(value: bytes | str) -> str:
    return value.decode("utf-8", "replace") if isinstance(value, bytes) else value


def run_direction(
    sender: paramiko.SSHClient,
    sender_password: str,
    sender_port: str,
    receiver: paramiko.SSHClient,
    receiver_password: str,
    receiver_port: str,
    payloads: list[str],
) -> list[str]:
    stdout, stderr = start_sudo_python(receiver, receiver_password, receiver_code(receiver_port, len(payloads), 8.0))
    if text_of(stdout.readline()).strip() != "READY":
        raise RuntimeError(text_of(stderr.read()))
    time.sleep(0.5)
    tx_out, tx_err = start_sudo_python(sender, sender_password, sender_code(sender_port, payloads))
    tx_text = text_of(tx_out.read())
    tx_error = text_of(tx_err.read())
    rx_text = text_of(stdout.read())
    rx_error = text_of(stderr.read())
    print(tx_text, end="")
    print(rx_text, end="")
    if tx_error or rx_error:
        raise RuntimeError((tx_error + rx_error).strip())
    return [line[3:] for line in rx_text.splitlines() if line.startswith("RX ")]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=3)
    args = parser.parse_args()
    if not 1 <= args.count <= 5:
        parser.error("--count must be 1..5")

    ground_spec, car_spec = targets()
    ground_password = os.environ.get("GROUND_STATION_PASSWORD")
    car_password = os.environ.get("ROCK5A_PASSWORD")
    if not ground_password or not car_password:
        parser.error("GROUND_STATION_PASSWORD and ROCK5A_PASSWORD must be set")

    run_id = uuid.uuid4().hex[:8]
    ground = connect(ground_spec[0], ground_spec[1], ground_password)
    car = connect(car_spec[0], car_spec[1], car_password)
    try:
        g2c = [f"G2C {run_id} {index:02d}" for index in range(args.count)]
        c2g = [f"C2G {run_id} {index:02d}" for index in range(args.count)]
        received_g2c = run_direction(
            ground, ground_password, ground_spec[2],
            car, car_password, car_spec[2], g2c,
        )
        received_c2g = run_direction(
            car, car_password, car_spec[2],
            ground, ground_password, ground_spec[2], c2g,
        )
    finally:
        ground.close(); car.close()

    g2c_ok = set(received_g2c) == set(g2c)
    c2g_ok = set(received_c2g) == set(c2g)
    print(f"GROUND_TO_CAR={len(received_g2c)}/{len(g2c)}")
    print(f"CAR_TO_GROUND={len(received_c2g)}/{len(c2g)}")
    print("RESULT: BIDIRECTIONAL LINK CONFIRMED" if g2c_ok and c2g_ok else "RESULT: LINK INCOMPLETE")
    return 0 if g2c_ok and c2g_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
