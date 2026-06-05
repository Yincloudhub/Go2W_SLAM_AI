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
        self.assertIn("语义视觉未启动或离线，不参与运动决策", web.INDEX_HTML)
        self.assertIn("近场运动许可仍由实时深度摘要与 SLAM 安全门决定", web.INDEX_HTML)
        self.assertIn("运动安全门", web.INDEX_HTML)
        self.assertIn("外部边缘节点", web.INDEX_HTML)
        self.assertIn('id="m-capture"', web.INDEX_HTML)
        self.assertIn('id="m-notwired"', web.INDEX_HTML)
        self.assertNotIn("鏈繛鎺", web.INDEX_HTML)

    def test_operator_ui_busy_state_does_not_lock_text_inputs(self):
        self.assertIn('document.querySelectorAll("button")', web.INDEX_HTML)
        self.assertNotIn('document.querySelectorAll("button, input, textarea, select")', web.INDEX_HTML)

    def test_operator_ui_has_topology_registry_controls(self):
        self.assertIn("/api/topology", web.INDEX_HTML)
        self.assertIn("topology-node", web.INDEX_HTML)
        self.assertIn("topology-registry-add", web.INDEX_HTML)
        self.assertIn("现场验证", web.INDEX_HTML)
        self.assertIn("一键就绪：雷达 + SLAM", web.INDEX_HTML)
        self.assertIn('id="exec-toggle"', web.INDEX_HTML)
        self.assertIn('id="weak-toggle"', web.INDEX_HTML)
        self.assertNotIn('id="topology-add"', web.INDEX_HTML)
        self.assertNotIn('id="exec-on"', web.INDEX_HTML)
        self.assertNotIn('id="weak-on"', web.INDEX_HTML)
        self.assertNotIn('data-action="current"', web.INDEX_HTML)
        self.assertIn("保存备用锚点", web.INDEX_HTML)
        self.assertIn("校准初始点", web.INDEX_HTML)
        self.assertIn("更新坐标", web.INDEX_HTML)

    def test_history_keeps_compact_failure_reason(self):
        state = web.WebState()
        state.remember(
            "go target",
            {
                "exit_code": 3,
                "accepted": False,
                "stdout": "prefix\nverification_guard: target requires calibration\n",
                "stderr": "warning: topic not confirmed yet: /slam_info\n",
                "summary": {"safety": "ok"},
            },
        )
        item = state.snapshot()["history"][0]
        self.assertIn("verification_guard", item["reason"])
        self.assertIn("/slam_info", item["reason"])

    def test_topology_soft_delete_and_restore_updates_registry(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            registry_path = root / "registry.json"
            registry_path.write_text(
                json.dumps(
                    {
                        "maps": [
                            {
                                "map_id": "go2w_real_site",
                                "topology_nodes": [
                                    {"node_id": "wp_a", "name": "A", "tags": ["real_site"], "pose": {"x": 1.0, "y": 2.0}}
                                ],
                            }
                        ]
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
                registry_path=registry_path,
            )
            app = web.OperatorWebApp(config)

            disabled = app.set_topology_node_disabled("wp_a", True, confirmed=True)
            self.assertTrue(disabled["accepted"])
            self.assertTrue(disabled["topology"]["nodes"][0]["disabled"])

            restored = app.set_topology_node_disabled("wp_a", False, confirmed=True)
            self.assertTrue(restored["accepted"])
            self.assertFalse(restored["topology"]["nodes"][0]["disabled"])

    def test_add_current_topology_node_uses_safe_live_pose(self):
        class FakeApp(web.OperatorWebApp):
            def status(self, *, force=False):
                return {"summary": {"loc": "true", "safety": "ok", "pose:x": "x=1.25, y=-0.50, yaw=0.30"}}

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            registry_path = root / "registry.json"
            registry_path.write_text(
                json.dumps({"maps": [{"map_id": "go2w_real_site", "topology_nodes": []}]}),
                encoding="utf-8",
            )
            app = FakeApp(
                web.WebConfig(
                    repo_root=root,
                    panel_bin=root / "missing",
                    gateway_client="missing",
                    start_slam_script="missing",
                    start_rviz2_script="missing",
                    registry_path=registry_path,
                )
            )

            result = app.add_current_topology_node({"node_id": "wp_new", "name": "New", "aliases": "New,新点"}, confirmed=True)
            self.assertTrue(result["accepted"])
            node = result["topology"]["nodes"][0]
            self.assertEqual(node["node_id"], "wp_new")
            self.assertIn("needs_standing_verification", node["tags"])
            self.assertAlmostEqual(node["pose"]["x"], 1.25)

    def test_protected_initial_point_cannot_be_overwritten_or_disabled(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            registry_path = root / "registry.json"
            registry_path.write_text(
                json.dumps({"maps": [{"map_id": "go2w_real_site", "topology_nodes": [{"node_id": "initial_point", "tags": [], "pose": {"x": 0, "y": 0}}]}]}),
                encoding="utf-8",
            )
            app = web.OperatorWebApp(
                web.WebConfig(
                    repo_root=root,
                    panel_bin=root / "missing",
                    gateway_client="missing",
                    start_slam_script="missing",
                    start_rviz2_script="missing",
                    registry_path=registry_path,
                )
            )
            overwrite = app.add_current_topology_node({"node_id": "initial_point", "name": "bad"}, confirmed=True)
            disable = app.set_topology_node_disabled("initial_point", True, confirmed=True)
            self.assertFalse(overwrite["accepted"])
            self.assertFalse(disable["accepted"])

    def test_protected_initial_point_can_be_recalibrated_only_through_verification(self):
        class FakeApp(web.OperatorWebApp):
            def status(self, *, force=False):
                return {"summary": {"loc": "true", "safety": "ok", "pose:x": "x=0.10, y=0.20, yaw=0.30"}}

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            registry_path = root / "registry.json"
            registry_path.write_text(
                json.dumps({"maps": [{"map_id": "go2w_real_site", "topology_nodes": [{"node_id": "initial_point", "tags": ["safe_return"], "pose": {"x": 0, "y": 0}}]}]}),
                encoding="utf-8",
            )
            app = FakeApp(
                web.WebConfig(
                    repo_root=root,
                    panel_bin=root / "missing",
                    gateway_client="missing",
                    start_slam_script="missing",
                    start_rviz2_script="missing",
                    registry_path=registry_path,
                )
            )
            result = app.verify_topology_node("initial_point", confirmed=True)
            self.assertTrue(result["accepted"])
            node = result["topology"]["nodes"][0]
            self.assertAlmostEqual(node["pose"]["x"], 0.10)
            self.assertAlmostEqual(node["pose"]["y"], 0.20)
            self.assertAlmostEqual(node["pose"]["yaw"], 0.30)
            self.assertIn("safe_return", node["tags"])

    def test_verify_topology_node_removes_safety_tags_when_robot_is_near(self):
        class FakeApp(web.OperatorWebApp):
            def status(self, *, force=False):
                return {"summary": {"loc": "true", "safety": "ok", "pose:x": "x=1.05, y=2.02, yaw=0.10"}}

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            registry_path = root / "registry.json"
            registry_path.write_text(
                json.dumps(
                    {
                        "maps": [
                            {
                                "map_id": "go2w_real_site",
                                "topology_nodes": [
                                    {
                                        "node_id": "wp_a",
                                        "name": "A",
                                        "tags": ["real_site", "needs_calibration", "needs_standing_verification"],
                                        "pose": {"x": 1.0, "y": 2.0},
                                    }
                                ],
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            app = FakeApp(
                web.WebConfig(
                    repo_root=root,
                    panel_bin=root / "missing",
                    gateway_client="missing",
                    start_slam_script="missing",
                    start_rviz2_script="missing",
                    registry_path=registry_path,
                )
            )
            result = app.verify_topology_node("wp_a", confirmed=True)
            self.assertTrue(result["accepted"])
            tags = result["topology"]["nodes"][0]["tags"]
            self.assertNotIn("needs_calibration", tags)
            self.assertNotIn("needs_standing_verification", tags)
            self.assertIn("ui_verified", tags)
            node = result["topology"]["nodes"][0]
            self.assertAlmostEqual(node["pose"]["x"], 1.05)
            self.assertAlmostEqual(node["pose"]["y"], 2.02)
            self.assertAlmostEqual(node["pose"]["yaw"], 0.10)
            registry = json.loads(registry_path.read_text(encoding="utf-8"))
            verified = registry["maps"][0]["topology_nodes"][0]
            self.assertAlmostEqual(verified["verification"]["previous_pose"]["x"], 1.0)
            self.assertAlmostEqual(verified["verification"]["previous_pose"]["y"], 2.0)

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

    def test_execute_on_is_blocked_when_live_localization_is_not_ready(self):
        class FakeApp(web.OperatorWebApp):
            def status(self, *, force=False):
                return {"summary": {"loc": "false", "safety": "slam_health_failed"}}

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            app = FakeApp(
                web.WebConfig(
                    repo_root=root,
                    panel_bin=root / "missing",
                    gateway_client="missing",
                    start_slam_script="missing",
                    start_rviz2_script="missing",
                )
            )
            result = app.command("/execute on", confirmed=True)
            self.assertFalse(result["accepted"])
            self.assertFalse(app.state.execute_enabled)
            self.assertIn("localization", result["stderr"])

    def test_execute_on_requires_fresh_stereo_motion_guard(self):
        class FakeApp(web.OperatorWebApp):
            def status(self, *, force=False):
                return {"summary": {"loc": "true", "safety": "ok"}}

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary_path = root / "artifacts" / "stereo_depth_summary.json"
            summary_path.parent.mkdir(parents=True)
            config = web.WebConfig(
                repo_root=root,
                panel_bin=root / "missing",
                gateway_client="missing",
                start_slam_script="missing",
                start_rviz2_script="missing",
            )
            app = FakeApp(config)

            missing = app.command("/execute on", confirmed=True)
            self.assertFalse(missing["accepted"])
            self.assertIn("offline_or_not_started", missing["stderr"])

            summary_path.write_text(
                json.dumps(
                    {
                        "timestamp_ms": int(time.time() * 1000),
                        "source": "stereo_depth",
                        "front_clearance_m": 2.0,
                        "left_clearance_m": 2.0,
                        "right_clearance_m": 2.0,
                        "roi_confidence": {"front": 0.8, "left": 0.8, "right": 0.8},
                    }
                ),
                encoding="utf-8",
            )
            allowed = app.command("/execute on", confirmed=True)
            self.assertTrue(allowed["accepted"])
            self.assertTrue(app.state.execute_enabled)
            app.command("/execute off")

            data = json.loads(summary_path.read_text(encoding="utf-8"))
            data["timestamp_ms"] = int(time.time() * 1000)
            data["right_clearance_m"] = 0.6
            summary_path.write_text(json.dumps(data), encoding="utf-8")
            blocked = app.command("/execute on", confirmed=True)
            self.assertFalse(blocked["accepted"])
            self.assertIn("right_obstacle_too_close", blocked["stderr"])

    def test_relocate_is_blocked_when_localization_is_already_healthy(self):
        class FakeApp(web.OperatorWebApp):
            def status(self, *, force=False):
                return {"summary": {"loc": "true", "safety": "ok"}}

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            app = FakeApp(
                web.WebConfig(
                    repo_root=root,
                    panel_bin=root / "missing",
                    gateway_client="missing",
                    start_slam_script="missing",
                    start_rviz2_script="missing",
                )
            )
            result = app.command("/relocate mapping_origin confirm", confirmed=True)
            self.assertFalse(result["accepted"])
            self.assertIn("already healthy", result["stderr"])

    def test_relocate_has_retry_cooldown(self):
        class FakeApp(web.OperatorWebApp):
            def status(self, *, force=False):
                return {"summary": {"loc": "false", "safety": "slam_health_failed"}}

            def run_panel_session(self, lines):
                return {"accepted": True, "exit_code": 0, "stdout": "", "stderr": "", "summary": {}}

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            app = FakeApp(
                web.WebConfig(
                    repo_root=root,
                    panel_bin=root / "missing",
                    gateway_client="missing",
                    start_slam_script="missing",
                    start_rviz2_script="missing",
                )
            )
            self.assertTrue(app.command("/relocate mapping_origin confirm", confirmed=True)["accepted"])
            result = app.command("/relocate mapping_origin confirm", confirmed=True)
            self.assertFalse(result["accepted"])
            self.assertIn("wait", result["stderr"])

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
            "--registry", "configs/maps/go2w_real_site_map_registry.json",
            "--map-id", "go2w_real_site",
            "--capture-command", "python3 scripts/capture_keyframe.py",
        ])
        argv = config.panel_argv(execute_enabled=True, weak_link_mode=True, current_node="wp_a")
        self.assertIn("--execute", argv)
        self.assertIn("--weak", argv)
        self.assertIn("wp_a", argv)
        self.assertIn("/tmp/slam_llm_command_client", argv)
        self.assertIn("--gateway-startup-wait-s", argv)
        self.assertIn("1.25", argv)
        self.assertIn("--registry", argv)
        registry_arg = argv[argv.index("--registry") + 1]
        self.assertEqual(
            Path(registry_arg),
            Path("configs/maps/go2w_real_site_map_registry.json"),
        )
        self.assertIn("--map-id", argv)
        self.assertIn("--capture-command", argv)
        self.assertIn("python3 scripts/capture_keyframe.py", argv)
        self.assertIn("go2w_real_site", argv)

    def test_capabilities_reflect_capture_command(self):
        config = web.make_config([
            "--repo-root", str(Path(__file__).resolve().parents[1]),
            "--panel-bin", "/tmp/go2w_operator_panel",
            "--capture-command", "python3 scripts/capture_keyframe.py",
        ])
        app = web.OperatorWebApp(config)

        capabilities = app.capabilities()

        self.assertTrue(capabilities["capture_keyframe"]["configured"])
        self.assertEqual(capabilities["capture_keyframe"]["status"], "ready")
        self.assertEqual(capabilities["relative_motion"]["status"], "not_wired")

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

    def test_edge_summary_is_semantic_only_until_explicit_safety_adapter_exists(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary_path = root / "artifacts" / "edge_perception_summary.json"
            summary_path.parent.mkdir(parents=True)
            summary_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "node_id": "nx_ti_radar_01",
                        "sensor_type": "ti_millimeter_wave_radar",
                        "source": "nx_ti_radar",
                        "timestamp_ms": int(time.time() * 1000),
                        "health": {"status": "ok"},
                        "observations": [{"track_id": str(index)} for index in range(40)],
                        "policy": {
                            "mode": "semantic_only",
                            "calibrated": True,
                            "safety_candidate": True,
                        },
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
                edge_summary_path=Path("artifacts/edge_perception_summary.json"),
            )

            loaded = web.OperatorWebApp(config).edge_summary()

            self.assertTrue(loaded["available"])
            self.assertTrue(loaded["fresh"])
            self.assertTrue(loaded["eligible_for_llm"])
            self.assertTrue(loaded["safety_candidate"])
            self.assertFalse(loaded["safety_wired"])
            self.assertEqual(len(loaded["data"]["observations"]), 32)

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
