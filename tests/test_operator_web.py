import importlib.util
import sys
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
        ])
        argv = config.panel_argv(execute_enabled=True, weak_link_mode=True, current_node="wp_a")
        self.assertIn("--execute", argv)
        self.assertIn("--weak", argv)
        self.assertIn("wp_a", argv)
        self.assertIn("/tmp/slam_llm_command_client", argv)


if __name__ == "__main__":
    unittest.main()
