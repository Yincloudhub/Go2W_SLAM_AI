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
                                        ],
                                        "llm_feedback_results": [
                                            {"text": "已到达701门口，任务完成。", "source": "local_llm"}
                                        ],
                                        "performance": {
                                            "poll_overruns": 1,
                                            "max_loop_elapsed_s": 0.42,
                                            "dropped_counts": {"arrival_samples": 2, "operator_feedback": 0, "llm_feedback": 0},
                                        },
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
        self.assertEqual(summary["llm_feedback_count"], 1)
        self.assertEqual(summary["llm_feedback_latest"], "已到达701门口，任务完成。")
        self.assertEqual(summary["llm_feedback_source"], "local_llm")
        self.assertEqual(summary["runtime_poll_overruns"], 1)
        self.assertEqual(summary["runtime_max_loop_elapsed_s"], 0.42)
        self.assertEqual(summary["runtime_dropped_counts"]["arrival_samples"], 2)


if __name__ == "__main__":
    unittest.main()
