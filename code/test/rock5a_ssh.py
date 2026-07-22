"""Small SSH helper for temporary diagnostics on the ROCK 5A.

Set ROCK5A_PASSWORD in the environment; the password is deliberately not kept
in this file.  The command is sent verbatim as a single shell command.
"""

from __future__ import annotations

import argparse
import os
import sys

import paramiko


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "command", nargs=argparse.REMAINDER, help="command to run on the ROCK 5A"
    )
    parser.add_argument(
        "--sudo", action="store_true", help="run the remote command through sudo"
    )
    parser.add_argument(
        "--upload",
        nargs=2,
        metavar=("LOCAL_FILE", "REMOTE_FILE"),
        help="copy one local file to the ROCK 5A before running the command",
    )
    args = parser.parse_args()

    password = os.environ.get("ROCK5A_PASSWORD")
    if not password:
        parser.error("ROCK5A_PASSWORD is not set")

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(
            hostname="192.168.31.224",
            username="radxa",
            password=password,
            timeout=10,
            banner_timeout=10,
            auth_timeout=10,
        )
        if args.upload:
            local_file, remote_file = args.upload
            sftp = client.open_sftp()
            try:
                sftp.put(local_file, remote_file)
            finally:
                sftp.close()

        if not args.command:
            return 0

        remote_command = " ".join(args.command)
        if args.sudo:
            remote_command = f"sudo -S -p '' sh -c {remote_command!r}"
        stdin, stdout, stderr = client.exec_command(remote_command)
        if args.sudo:
            stdin.write(password + "\n")
            stdin.flush()
            stdin.channel.shutdown_write()
        sys.stdout.write(stdout.read().decode(errors="replace"))
        sys.stderr.write(stderr.read().decode(errors="replace"))
        return stdout.channel.recv_exit_status()
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
