import importlib.util
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "go2w_web_tunnel.py"
    spec = importlib.util.spec_from_file_location("go2w_web_tunnel", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


tunnel = load_module()


class Go2wWebTunnelTests(unittest.TestCase):
    def test_make_config_does_not_embed_robot_password(self):
        config = tunnel.make_config([])

        self.assertEqual(config.ssh_password, "")

    def test_make_config_accepts_port_and_hosts(self):
        config = tunnel.make_config([
            "--ssh-host",
            "192.168.123.18",
            "--local-port",
            "9876",
            "--remote-port",
            "8765",
        ])

        self.assertEqual(config.ssh_host, "192.168.123.18")
        self.assertEqual(config.local_port, 9876)
        self.assertEqual(config.remote_port, 8765)

    def test_make_config_accepts_auto_host_candidates(self):
        config = tunnel.make_config([
            "--ssh-host",
            "auto",
            "--ssh-hosts",
            "192.168.3.17, 192.168.123.18,192.168.3.17",
        ])

        self.assertEqual(
            tunnel.ssh_host_candidates(config),
            ("192.168.3.17", "192.168.123.18"),
        )

    def test_explicit_host_bypasses_auto_candidates(self):
        config = tunnel.TunnelConfig(
            ssh_host="10.0.0.7",
            ssh_hosts=("192.168.3.17", "192.168.123.18"),
        )

        self.assertEqual(tunnel.ssh_host_candidates(config), ("10.0.0.7",))

    def test_open_ssh_falls_back_to_next_candidate(self):
        attempts = []

        class FakeClient:
            def set_missing_host_key_policy(self, policy):
                self.policy = policy

            def connect(self, host, **kwargs):
                attempts.append(host)
                if host == "192.168.123.18":
                    raise OSError("unreachable")

            def close(self):
                pass

        config = tunnel.TunnelConfig(
            ssh_host="auto",
            ssh_hosts=("192.168.123.18", "192.168.3.17"),
        )
        with patch.object(tunnel.paramiko, "SSHClient", FakeClient):
            client, selected_host = tunnel.open_ssh(config)

        self.assertIsInstance(client, FakeClient)
        self.assertEqual(selected_host, "192.168.3.17")
        self.assertEqual(attempts, ["192.168.123.18", "192.168.3.17"])

    def test_connection_manager_reconnects_after_transport_goes_inactive(self):
        transports = []
        clients = []

        class FakeTransport:
            def __init__(self):
                self.active = True

            def is_active(self):
                return self.active

        class FakeClient:
            def __init__(self, transport):
                self.transport = transport
                self.closed = False

            def get_transport(self):
                return self.transport

            def close(self):
                self.closed = True

        def fake_open_ssh(config):
            transport = FakeTransport()
            client = FakeClient(transport)
            transports.append(transport)
            clients.append(client)
            return client, f"host-{len(clients)}"

        manager = tunnel.SshConnectionManager(tunnel.TunnelConfig())
        with patch.object(tunnel, "open_ssh", fake_open_ssh):
            first = manager.get_transport()
            first.active = False
            second = manager.get_transport()

        self.assertIsNot(first, second)
        self.assertTrue(clients[0].closed)
        self.assertFalse(clients[1].closed)
        self.assertEqual(manager.selected_ssh_host, "host-2")
        manager.close()
        self.assertTrue(clients[1].closed)


if __name__ == "__main__":
    unittest.main()
