#!/usr/bin/env python3
"""Local TCP tunnel for the robot-side GO2W Web UI.

It forwards a local port, default 127.0.0.1:8765, to the robot's
127.0.0.1:8765 over SSH using Paramiko. This avoids exposing the Web UI on the
robot LAN while keeping the browser URL simple.
"""

from __future__ import annotations

import argparse
import os
import select
import socket
import socketserver
import sys
import threading
from dataclasses import dataclass
from typing import Optional

import paramiko


@dataclass(frozen=True)
class TunnelConfig:
    ssh_host: str = "192.168.123.18"
    ssh_user: str = "unitree"
    ssh_password: str = "123"
    local_host: str = "127.0.0.1"
    local_port: int = 8765
    remote_host: str = "127.0.0.1"
    remote_port: int = 8765


class ForwardServer(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True


class ForwardHandler(socketserver.BaseRequestHandler):
    ssh_transport: paramiko.Transport
    remote_host: str
    remote_port: int

    def handle(self) -> None:
        try:
            chan = self.ssh_transport.open_channel(
                "direct-tcpip",
                (self.remote_host, self.remote_port),
                self.request.getpeername(),
            )
        except Exception as exc:  # noqa: BLE001
            print(f"open_channel_failed={exc}", file=sys.stderr)
            return
        if chan is None:
            print("open_channel_failed=none", file=sys.stderr)
            return
        try:
            while True:
                readable, _, _ = select.select([self.request, chan], [], [], 1.0)
                if self.request in readable:
                    data = self.request.recv(16384)
                    if not data:
                        break
                    chan.sendall(data)
                if chan in readable:
                    data = chan.recv(16384)
                    if not data:
                        break
                    self.request.sendall(data)
        finally:
            chan.close()
            self.request.close()


def make_config(argv: Optional[list[str]] = None) -> TunnelConfig:
    parser = argparse.ArgumentParser(description="Forward local browser traffic to GO2W robot Web UI")
    parser.add_argument("--ssh-host", default=os.environ.get("GO2W_SSH_HOST", "192.168.123.18"))
    parser.add_argument("--ssh-user", default=os.environ.get("GO2W_SSH_USER", "unitree"))
    parser.add_argument("--ssh-password", default=os.environ.get("GO2W_SSH_PASSWORD", "123"))
    parser.add_argument("--local-host", default=os.environ.get("GO2W_TUNNEL_LOCAL_HOST", "127.0.0.1"))
    parser.add_argument("--local-port", type=int, default=int(os.environ.get("GO2W_TUNNEL_LOCAL_PORT", "8765")))
    parser.add_argument("--remote-host", default=os.environ.get("GO2W_TUNNEL_REMOTE_HOST", "127.0.0.1"))
    parser.add_argument("--remote-port", type=int, default=int(os.environ.get("GO2W_TUNNEL_REMOTE_PORT", "8765")))
    args = parser.parse_args(argv)
    return TunnelConfig(
        ssh_host=args.ssh_host,
        ssh_user=args.ssh_user,
        ssh_password=args.ssh_password,
        local_host=args.local_host,
        local_port=args.local_port,
        remote_host=args.remote_host,
        remote_port=args.remote_port,
    )


def open_ssh(config: TunnelConfig) -> paramiko.SSHClient:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        config.ssh_host,
        username=config.ssh_user,
        password=config.ssh_password,
        timeout=8,
        banner_timeout=8,
        auth_timeout=8,
        look_for_keys=False,
        allow_agent=False,
    )
    return client


def run_tunnel(config: TunnelConfig) -> None:
    ssh = open_ssh(config)
    transport = ssh.get_transport()
    if transport is None:
        raise RuntimeError("ssh transport unavailable")

    class Handler(ForwardHandler):
        ssh_transport = transport
        remote_host = config.remote_host
        remote_port = config.remote_port

    server = ForwardServer((config.local_host, config.local_port), Handler)
    print(
        f"forwarding http://{config.local_host}:{config.local_port} "
        f"-> {config.ssh_user}@{config.ssh_host}:{config.remote_host}:{config.remote_port}",
        flush=True,
    )
    try:
        server.serve_forever()
    finally:
        server.server_close()
        ssh.close()


def main(argv: Optional[list[str]] = None) -> int:
    config = make_config(argv)
    run_tunnel(config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
