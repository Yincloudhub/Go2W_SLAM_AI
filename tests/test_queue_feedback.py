import unittest
from types import SimpleNamespace

from scripts.run_robot_closed_loop import generate_llm_feedback_result, operator_feedback_message


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


if __name__ == "__main__":
    unittest.main()
