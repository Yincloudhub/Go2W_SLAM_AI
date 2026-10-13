import json
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

from edge_autonomy.mission_agent import (
    build_mission_preview,
    call_openai_compatible_chat,
    handle_agent_message,
)


def write_registry(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "maps": [
                    {
                        "map_id": "test_map",
                        "topology_nodes": [
                            {
                                "node_id": "initial_point",
                                "name": "Initial Point",
                                "aliases": ["start"],
                            },
                            {
                                "node_id": "hall",
                                "name": "Hall",
                                "aliases": ["middle"],
                            },
                            {
                                "node_id": "yin_station",
                                "name": "Yin Station",
                                "aliases": ["yin"],
                            },
                        ],
                        "topology_edges": [
                            {"from": "initial_point", "to": "hall", "bidirectional": True},
                            {"from": "hall", "to": "yin_station", "bidirectional": True},
                        ],
                    }
                ]
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


class FakeChatHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0") or "0")
        self.server.last_body = json.loads(self.rfile.read(length).decode("utf-8"))
        body = json.dumps(
            {
                "choices": [
                    {
                        "message": {
                            "content": "cloud reply ok",
                        }
                    }
                ]
            }
        ).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        return


class MissionAgentTests(unittest.TestCase):
    def test_chat_preview_does_not_create_motion(self):
        plan = build_mission_preview("hello, what can you do?", map_id="test_map")

        self.assertEqual(plan["intent_kind"], "chat")
        self.assertFalse(plan["execution_enabled"])
        self.assertFalse(plan["llm_direct_motion"])
        self.assertEqual(plan["steps"][0]["type"], "chat")

    def test_motion_and_photo_preview_stays_readonly(self):
        with tempfile.TemporaryDirectory() as tmp:
            registry = Path(tmp) / "registry.json"
            write_registry(registry)

            plan = build_mission_preview(
                "go from start to yin and take photo",
                registry_path=str(registry),
                map_id="test_map",
            )

        motion_steps = [step for step in plan["steps"] if step["type"] == "motion"]
        behavior_steps = [step for step in plan["steps"] if step["type"] == "behavior"]
        self.assertEqual(plan["intent_kind"], "mixed")
        self.assertEqual(motion_steps[0]["route"], ["initial_point", "hall", "yin_station"])
        self.assertEqual(motion_steps[0]["status"], "validated")
        self.assertFalse(motion_steps[0]["execution_enabled"])
        self.assertEqual(behavior_steps[0]["behavior"], "capture_photo")

    def test_cloud_provider_uses_openai_compatible_chat_endpoint(self):
        server = HTTPServer(("127.0.0.1", 0), FakeChatHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(thread.join, 2)
        self.addCleanup(server.shutdown)
        host, port = server.server_address

        result = call_openai_compatible_chat(
            "hello",
            base_url="http://%s:%s/v1" % (host, port),
            api_key="test-key",
            model="test-model",
            timeout_s=2,
        )

        self.assertTrue(result["ok"])
        self.assertEqual(result["reply"], "cloud reply ok")
        self.assertEqual(server.last_body["model"], "test-model")
        self.assertEqual(server.last_body["messages"][-1]["content"], "hello")

    def test_handle_agent_message_returns_reply_and_plan(self):
        server = HTTPServer(("127.0.0.1", 0), FakeChatHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(thread.join, 2)
        self.addCleanup(server.shutdown)
        host, port = server.server_address

        with tempfile.TemporaryDirectory() as tmp:
            registry = Path(tmp) / "registry.json"
            write_registry(registry)
            result = handle_agent_message(
                "go from start to yin",
                provider="cloud",
                registry_path=str(registry),
                map_id="test_map",
                cloud_base_url="http://%s:%s/v1" % (host, port),
                cloud_api_key="test-key",
                cloud_model="test-model",
                timeout_s=2,
            )

        self.assertTrue(result["ok"])
        self.assertEqual(result["reply"], "cloud reply ok")
        self.assertFalse(result["execution_enabled"])
        self.assertEqual(result["mission_plan"]["intent_kind"], "motion")


if __name__ == "__main__":
    unittest.main()
