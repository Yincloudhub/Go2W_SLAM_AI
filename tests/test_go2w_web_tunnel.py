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


if __name__ == "__main__":
    unittest.main()
