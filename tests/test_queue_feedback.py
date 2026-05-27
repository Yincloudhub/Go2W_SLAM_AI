import unittest
from types import SimpleNamespace

from scripts.run_robot_closed_loop import generate_llm_feedback_result, operator_feedback_message


class FailingBackend:
    def generate(self, prompt: str, *, system_prompt: str, max_tokens: int, timeout_s: int) -> str:
        raise AssertionError("live LLM should not run for progress feedback by default")


class QueueFeedbackTests(unittest.TestCase):
    def test_operator_feedback_message_is_ui_and_llm_ready(self) -> None:
        message = operator_feedback_message(
            "progress",
            "正在前往701门口",
            target_node="room_701_door",
            target_name="701门口",
            distance_m=1.25,
        )

        self.assertEqual(message["channel"], "operator_display")
        self.assertTrue(message["llm_surface"])
        self.assertEqual(message["target_name"], "701门口")
        self.assertEqual(message["distance_to_target_m"], 1.25)

    def test_template_llm_feedback_result_is_display_ready(self) -> None:
        request = {
            "phase": "arrived",
            "target_node": "room_701_door",
            "target_name": "701门口",
            "distance_to_target_m": 0.18,
        }
        result = generate_llm_feedback_result(request, SimpleNamespace(llm_feedback_mode="template"))

        self.assertIsNotNone(result)
        self.assertEqual(result["source"], "template")
        self.assertIn("已到达701门口", result["text"])
        self.assertTrue(result["llm_surface"])

    def test_live_llm_progress_is_deferred_by_default(self) -> None:
        request = {
            "phase": "progress",
            "target_node": "room_701_door",
            "target_name": "701门口",
            "distance_to_target_m": 1.25,
        }
        args = SimpleNamespace(
            llm_feedback_mode="live",
            llm_feedback_live_progress=False,
            local_command="unused",
            llm_feedback_system="system",
            llm_feedback_max_tokens=16,
            llm_feedback_timeout_s=1,
        )

        result = generate_llm_feedback_result(request, args, backend=FailingBackend())

        self.assertEqual(result["source"], "template_progress_budget")
        self.assertTrue(result["live_deferred"])

    def test_live_llm_queued_feedback_is_deferred(self) -> None:
        request = {
            "phase": "queued",
            "target_node": "room_701_door",
            "target_name": "701门口",
        }
        args = SimpleNamespace(
            llm_feedback_mode="live",
            llm_feedback_live_progress=True,
            local_command="unused",
            llm_feedback_system="system",
            llm_feedback_max_tokens=16,
            llm_feedback_timeout_s=1,
        )

        result = generate_llm_feedback_result(request, args, backend=FailingBackend())

        self.assertEqual(result["source"], "template_queue_budget")
        self.assertTrue(result["live_deferred"])


if __name__ == "__main__":
    unittest.main()
