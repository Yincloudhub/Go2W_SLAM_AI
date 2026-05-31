import importlib.util
import sys
import unittest
from pathlib import Path


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


if __name__ == "__main__":
    unittest.main()
