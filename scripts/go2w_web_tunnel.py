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


DEFAULT_SSH_HOSTS = ("192.168.123.18", "192.168.3.17")


@dataclass(frozen=True)
class TunnelConfig:
    ssh_host: str = "auto"
    ssh_hosts: tuple[str, ...] = DEFAULT_SSH_HOSTS
    ssh_user: str = "unitree"
    ssh_password: str = "123"
    ssh_connect_timeout_s: float = 3.0
    local_host: str = "127.0.0.1"
    local_port: int = 8765
    remote_host: str = "127.0.0.1"
    remote_port: int = 8765


class ForwardServer(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True


class ForwardHandler(socketserver.BaseRequestHandler):
    ssh_manager: "SshConnectionManager"
    remote_host: str
    remote_port: int

    def handle(self) -> None:
        chan = None
        transport = None
        for attempt in range(2):
            try:
                transport = self.ssh_manager.get_transport()
                chan = transport.open_channel(
                    "direct-tcpip",
                    (self.remote_host, self.remote_port),
                    self.request.getpeername(),
                )
                break
            except Exception as exc:  # noqa: BLE001
                self.ssh_manager.invalidate(transport)
                print(f"open_channel_failed={exc}", file=sys.stderr)
                if attempt == 0:
                    print("retrying_with_fresh_ssh_transport", file=sys.stderr)
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
        except (EOFError, OSError, paramiko.SSHException) as exc:
            print(f"forwarding_interrupted={exc}", file=sys.stderr)
            if transport is not None and not transport.is_active():
                self.ssh_manager.invalidate(transport)
        finally:
            chan.close()
            self.request.close()


def split_hosts(value: str) -> tuple[str, ...]:
    return tuple(host.strip() for host in value.split(",") if host.strip())


def ssh_host_candidates(config: TunnelConfig) -> tuple[str, ...]:
    if config.ssh_host.strip().lower() not in ("", "auto"):
        return (config.ssh_host.strip(),)
    return tuple(dict.fromkeys(config.ssh_hosts))


def make_config(argv: Optional[list[str]] = None) -> TunnelConfig:
    parser = argparse.ArgumentParser(description="Forward local browser traffic to GO2W robot Web UI")
    parser.add_argument(
        "--ssh-host",
        default=os.environ.get("GO2W_SSH_HOST", "auto"),
        help="Explicit robot management address, or 'auto' to probe --ssh-hosts in order.",
    )
    parser.add_argument(
        "--ssh-hosts",
        default=os.environ.get("GO2W_SSH_HOSTS", ",".join(DEFAULT_SSH_HOSTS)),
        help="Comma-separated management addresses probed when --ssh-host=auto.",
    )
    parser.add_argument("--ssh-user", default=os.environ.get("GO2W_SSH_USER", "unitree"))
    parser.add_argument("--ssh-password", default=os.environ.get("GO2W_SSH_PASSWORD", "123"))
    parser.add_argument(
        "--ssh-connect-timeout-s",
        type=float,
        default=float(os.environ.get("GO2W_SSH_CONNECT_TIMEOUT_S", "3")),
    )
    parser.add_argument("--local-host", default=os.environ.get("GO2W_TUNNEL_LOCAL_HOST", "127.0.0.1"))
    parser.add_argument("--local-port", type=int, default=int(os.environ.get("GO2W_TUNNEL_LOCAL_PORT", "8765")))
    parser.add_argument("--remote-host", default=os.environ.get("GO2W_TUNNEL_REMOTE_HOST", "127.0.0.1"))
    parser.add_argument("--remote-port", type=int, default=int(os.environ.get("GO2W_TUNNEL_REMOTE_PORT", "8765")))
    args = parser.parse_args(argv)
    return TunnelConfig(
        ssh_host=args.ssh_host,
        ssh_hosts=split_hosts(args.ssh_hosts),
        ssh_user=args.ssh_user,
        ssh_password=args.ssh_password,
        ssh_connect_timeout_s=args.ssh_connect_timeout_s,
        local_host=args.local_host,
        local_port=args.local_port,
        remote_host=args.remote_host,
        remote_port=args.remote_port,
    )


def open_ssh(config: TunnelConfig) -> tuple[paramiko.SSHClient, str]:
    candidates = ssh_host_candidates(config)
    if not candidates:
        raise RuntimeError("no SSH management address candidates configured")

    errors: list[str] = []
    for ssh_host in candidates:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(
                ssh_host,
                username=config.ssh_user,
                password=config.ssh_password,
                timeout=config.ssh_connect_timeout_s,
                banner_timeout=config.ssh_connect_timeout_s,
                auth_timeout=config.ssh_connect_timeout_s,
                look_for_keys=False,
                allow_agent=False,
            )
        except Exception as exc:  # noqa: BLE001
            client.close()
            errors.append(f"{ssh_host}: {exc}")
            print(f"ssh_candidate_failed={ssh_host} error={exc}", file=sys.stderr)
            continue
        return client, ssh_host

    raise RuntimeError(f"unable to connect to GO2W SSH candidates: {'; '.join(errors)}")


class SshConnectionManager:
    """Keeps the browser tunnel usable across robot reboots and IP changes."""

    def __init__(self, config: TunnelConfig):
        self.config = config
        self._lock = threading.Lock()
        self._ssh: Optional[paramiko.SSHClient] = None
        self._transport: Optional[paramiko.Transport] = None
        self.selected_ssh_host: Optional[str] = None

    def get_transport(self) -> paramiko.Transport:
        with self._lock:
            if self._transport is not None and self._transport.is_active():
                return self._transport
            self._close_locked()
            ssh, selected_ssh_host = open_ssh(self.config)
            transport = ssh.get_transport()
            if transport is None or not transport.is_active():
                ssh.close()
                raise RuntimeError("ssh transport unavailable")
            self._ssh = ssh
            self._transport = transport
            self.selected_ssh_host = selected_ssh_host
            print(f"ssh_connected={selected_ssh_host}", flush=True)
            return transport

    def invalidate(self, transport: Optional[paramiko.Transport]) -> None:
        with self._lock:
            if transport is None or transport is self._transport:
                self._close_locked()

    def close(self) -> None:
        with self._lock:
            self._close_locked()

    def _close_locked(self) -> None:
        if self._ssh is not None:
            self._ssh.close()
        self._ssh = None
        self._transport = None
        self.selected_ssh_host = None


def run_tunnel(config: TunnelConfig) -> None:
    ssh_manager = SshConnectionManager(config)
    ssh_manager.get_transport()
    manager = ssh_manager

    class Handler(ForwardHandler):
        ssh_manager = manager
        remote_host = config.remote_host
        remote_port = config.remote_port

    server = ForwardServer((config.local_host, config.local_port), Handler)
    print(
        f"forwarding http://{config.local_host}:{config.local_port} "
        f"-> {config.ssh_user}@auto:{config.remote_host}:{config.remote_port}",
        flush=True,
    )
    try:
        server.serve_forever()
    finally:
        server.server_close()
        ssh_manager.close()


def main(argv: Optional[list[str]] = None) -> int:
    config = make_config(argv)
    run_tunnel(config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
