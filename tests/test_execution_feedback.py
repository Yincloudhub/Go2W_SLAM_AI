import unittest

from edge_autonomy.execution_report import summarize_agent_output


class ExecutionFeedbackTests(unittest.TestCase):
    def test_summary_extracts_latest_operator_feedback(self) -> None:
        output = {
            "steps": [
                {
                    "step": "closed_loop",
                    "result": {
                        "result": {
                            "queue_execution": {
                                "completed": True,
                                "events": [
                                    {
                                        "operator_feedback": [
                                            {"text": "正在前往701门口"},
                                            {"text": "已到达701门口，导航已暂停。"},
                                        ]
                                    }
                                ],
                            },
                            "planner": {},
                            "execution": {},
                        }
                    },
                }
            ]
        }

        summary = summarize_agent_output(output)

        self.assertEqual(summary["operator_feedback_count"], 2)
        self.assertEqual(summary["operator_feedback_latest"], "已到达701门口，导航已暂停。")


if __name__ == "__main__":
    unittest.main()
