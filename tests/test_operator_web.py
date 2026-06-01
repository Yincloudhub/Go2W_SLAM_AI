import importlib.util
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path


def load_operator_web():
    path = Path(__file__).resolve().parents[1] / "scripts" / "go2w_operator_web.py"
    spec = importlib.util.spec_from_file_location("go2w_operator_web", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


web = load_operator_web()


class OperatorWebTests(unittest.TestCase):
    def test_operator_ui_is_chinese_and_has_feedback_screen(self):
        self.assertIn("GO2W 多模态自主机器狗", web.INDEX_HTML)
        self.assertIn("智能巡检交互屏", web.INDEX_HTML)
        self.assertIn("机器狗回复", web.INDEX_HTML)
        self.assertIn("自适应：任务 2 秒 / 待机 5 秒", web.INDEX_HTML)
        self.assertIn('requestedMs === -1 ? (active.has(latestPhase) ? 2000 : 5000)', web.INDEX_HTML)
        self.assertIn("statusInFlight", web.INDEX_HTML)
        self.assertIn("commandInFlight", web.INDEX_HTML)
        self.assertIn("视觉数据过期，不参与决策", web.INDEX_HTML)
        self.assertIn("视觉侧车离线，系统保持雷达与 SLAM 闭环", web.INDEX_HTML)
        self.assertNotIn("鏈繛鎺", web.INDEX_HTML)

    def test_parse_panel_summary(self):
        text = "noise\n[18:38:24] phase=idle | target=none | loc=true | map=true | motion=false | safety=ok\n"
        parsed = web.parse_panel_summary(text)
        self.assertEqual(parsed["phase"], "idle")
        self.assertEqual(parsed["target"], "none")
        self.assertEqual(parsed["loc"], "true")
        self.assertEqual(parsed["motion"], "false")

    def test_execute_on_requires_confirmation(self):
        state = web.WebState()
        result = state.apply_local_setting("/execute on", confirmed=False)
        self.assertEqual(result["exit_code"], 2)
        self.assertFalse(state.execute_enabled)

        result = state.apply_local_setting("/execute on", confirmed=True)
        self.assertEqual(result["exit_code"], 0)
        self.assertTrue(state.execute_enabled)

    def test_local_state_commands(self):
        state = web.WebState(current_node="initial_point")
        self.assertEqual(state.apply_local_setting("/weak on")["exit_code"], 0)
        self.assertTrue(state.weak_link_mode)
        self.assertEqual(state.apply_local_setting("/current wp_a")["exit_code"], 0)
        self.assertEqual(state.current_node, "wp_a")
        self.assertIsNone(state.apply_local_setting("/status"))

    def test_panel_argv_carries_runtime_flags(self):
        config = web.make_config([
            "--repo-root", str(Path(__file__).resolve().parents[1]),
            "--panel-bin", "/tmp/go2w_operator_panel",
            "--gateway-client", "/tmp/slam_llm_command_client",
            "--interface", "eth0",
            "--current-node", "initial_point",
            "--gateway-startup-wait-s", "1.25",
        ])
        argv = config.panel_argv(execute_enabled=True, weak_link_mode=True, current_node="wp_a")
        self.assertIn("--execute", argv)
        self.assertIn("--weak", argv)
        self.assertIn("wp_a", argv)
        self.assertIn("/tmp/slam_llm_command_client", argv)
        self.assertIn("--gateway-startup-wait-s", argv)
        self.assertIn("1.25", argv)

    def test_stereo_summary_file_is_bounded_diagnostic(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary_path = root / "artifacts" / "stereo_depth_summary.json"
            summary_path.parent.mkdir(parents=True)
            summary_path.write_text(
                json.dumps({"timestamp_ms": 1, "source": "stereo_depth", "front_clearance_m": 1.2, "confidence": 0.8}),
                encoding="utf-8",
            )
            config = web.WebConfig(
                repo_root=root,
                panel_bin=root / "missing",
                gateway_client="missing",
                start_slam_script="missing",
                start_rviz2_script="missing",
                stereo_summary_path=Path("artifacts/stereo_depth_summary.json"),
                stereo_stale_ms=12345,
            )
            app = web.OperatorWebApp(config)
            loaded = app.stereo_summary()
            self.assertTrue(loaded["available"])
            self.assertEqual(loaded["data"]["source"], "stereo_depth")
            self.assertEqual(loaded["stale_ms"], 12345)
            self.assertTrue(loaded["stale_by_age"])

    def test_stereo_summary_stale_threshold_is_configurable(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary_path = root / "stereo.json"
            summary_path.write_text(
                json.dumps({"timestamp_ms": int(time.time() * 1000) - 2000, "source": "stereo_depth"}),
                encoding="utf-8",
            )
            config = web.WebConfig(
                repo_root=root,
                panel_bin=root / "missing",
                gateway_client="missing",
                start_slam_script="missing",
                start_rviz2_script="missing",
                stereo_summary_path=summary_path,
                stereo_stale_ms=5000,
            )
            loaded = web.OperatorWebApp(config).stereo_summary()
            self.assertFalse(loaded["stale_by_age"])

    def test_semantic_summary_file_is_bounded_diagnostic(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary_path = root / "artifacts" / "vision_semantic_summary.json"
            summary_path.parent.mkdir(parents=True)
            summary_path.write_text(
                json.dumps(
                    {
                        "timestamp_ms": 1,
                        "available": True,
                        "source": "deepyolo_realsense",
                        "object_count": 1,
                        "recommended_action": "slow_and_watch",
                    }
                ),
                encoding="utf-8",
            )
            config = web.WebConfig(
                repo_root=root,
                panel_bin=root / "missing",
                gateway_client="missing",
                start_slam_script="missing",
                start_rviz2_script="missing",
                semantic_summary_path=Path("artifacts/vision_semantic_summary.json"),
                semantic_stale_ms=2345,
            )
            loaded = web.OperatorWebApp(config).semantic_summary()
            self.assertTrue(loaded["available"])
            self.assertEqual(loaded["data"]["source"], "deepyolo_realsense")
            self.assertEqual(loaded["stale_ms"], 2345)
            self.assertTrue(loaded["stale_by_age"])

    def test_semantic_summary_prefers_source_file_age(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary_path = root / "artifacts" / "vision_semantic_summary.json"
            summary_path.parent.mkdir(parents=True)
            summary_path.write_text(
                json.dumps(
                    {
                        "timestamp_ms": 1,
                        "source_file_age_ms": 120,
                        "stale_ms": 3000,
                        "available": True,
                        "source": "deepyolo_realsense",
                        "source_status": "fresh",
                        "stale": False,
                    }
                ),
                encoding="utf-8",
            )
            config = web.WebConfig(
                repo_root=root,
                panel_bin=root / "missing",
                gateway_client="missing",
                start_slam_script="missing",
                start_rviz2_script="missing",
                semantic_summary_path=Path("artifacts/vision_semantic_summary.json"),
            )

            loaded = web.OperatorWebApp(config).semantic_summary()

            self.assertEqual(loaded["age_ms"], 120)
            self.assertFalse(loaded["stale_by_age"])

    def test_status_uses_short_cache_to_bound_panel_spawns(self):
        class FakeApp(web.OperatorWebApp):
            def __init__(self, config):
                super().__init__(config)
                self.calls = 0

            def run_panel_session(self, lines):
                self.calls += 1
                return {
                    "accepted": True,
                    "exit_code": 0,
                    "stdout": f"call={self.calls}\n",
                    "stderr": "",
                    "summary": {"phase": "idle"},
                }

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = web.WebConfig(
                repo_root=root,
                panel_bin=root / "missing",
                gateway_client="missing",
                start_slam_script="missing",
                start_rviz2_script="missing",
                status_cache_ms=10000,
            )
            app = FakeApp(config)
            first = app.status()
            second = app.status()
            forced = app.status(force=True)

            self.assertEqual(app.calls, 2)
            self.assertFalse(first["cache"]["hit"])
            self.assertTrue(second["cache"]["hit"])
            self.assertFalse(forced["cache"]["hit"])

    def test_command_invalidates_status_cache(self):
        class FakeApp(web.OperatorWebApp):
            def __init__(self, config):
                super().__init__(config)
                self.calls = 0

            def run_panel_session(self, lines):
                self.calls += 1
                return {
                    "accepted": True,
                    "exit_code": 0,
                    "stdout": "phase=idle | loc=false\n",
                    "stderr": "",
                    "summary": {"phase": "idle"},
                }

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = web.WebConfig(
                repo_root=root,
                panel_bin=root / "missing",
                gateway_client="missing",
                start_slam_script="missing",
                start_rviz2_script="missing",
                status_cache_ms=10000,
            )
            app = FakeApp(config)
            app.status()
            app.command("/weak on")
            app.status()

            self.assertEqual(app.calls, 2)


if __name__ == "__main__":
    unittest.main()
