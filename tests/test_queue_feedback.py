import unittest

from scripts.run_robot_closed_loop import operator_feedback_message


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


if __name__ == "__main__":
    unittest.main()
