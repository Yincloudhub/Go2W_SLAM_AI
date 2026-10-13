import importlib.util
import json
import sys
import threading
import unittest
import urllib.error
import urllib.request
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "cloud_llm_gateway.py"
    spec = importlib.util.spec_from_file_location("cloud_llm_gateway", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


cloud_gateway = load_module()


class CloudLlmGatewayTests(unittest.TestCase):
    def start_server(self):
        args = cloud_gateway.parse_args([
            "--host",
            "127.0.0.1",
            "--port",
            "0",
            "--repo",
            str(Path(__file__).resolve().parents[1]),
            "--token",
            "secret",
        ])
        server = cloud_gateway.make_server(args)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(thread.join, 2)
        self.addCleanup(server.shutdown)
        return server

    def url(self, server, path):
        host, port = server.server_address
        return f"http://{host}:{port}{path}"

    def test_health_requires_token(self):
        server = self.start_server()
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(self.url(server, "/health"), timeout=2)
        self.assertEqual(ctx.exception.code, 401)

    def test_health_accepts_bearer_token(self):
        server = self.start_server()
        request = urllib.request.Request(
            self.url(server, "/health"),
            headers={"Authorization": "Bearer secret"},
        )
        body = urllib.request.urlopen(request, timeout=2).read().decode("utf-8")
        data = json.loads(body)

        self.assertTrue(data["ok"])
        self.assertTrue(data["readonly"])
        self.assertFalse(data["execution_enabled"])

    def test_execution_endpoint_is_not_available(self):
        server = self.start_server()
        request = urllib.request.Request(
            self.url(server, "/navigate"),
            headers={"Authorization": "Bearer secret"},
        )
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(request, timeout=2)
        self.assertEqual(ctx.exception.code, 404)

    def test_agent_message_endpoint_returns_readonly_plan(self):
        server = self.start_server()
        body = json.dumps({"message": "hello", "provider": "cloud"}).encode("utf-8")
        request = urllib.request.Request(
            self.url(server, "/agent/message"),
            data=body,
            headers={
                "Authorization": "Bearer secret",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        data = json.loads(urllib.request.urlopen(request, timeout=2).read().decode("utf-8"))

        self.assertFalse(data["execution_enabled"])
        self.assertTrue(data["readonly"])
        self.assertEqual(data["mission_plan"]["intent_kind"], "chat")


if __name__ == "__main__":
    unittest.main()
