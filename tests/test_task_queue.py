import unittest

from edge_autonomy.task_queue import validate_task_queue


def make_queue() -> dict:
    return {
        "queue_id": "queue_test",
        "mode": "sequential",
        "status": "planned",
        "source": "semantic_topology",
        "targets": ["room701"],
        "steps": [
            {
                "task_id": "task_1",
                "action": "navigate",
                "target_node": "room701",
                "status": "pending",
                "requires_preflight": True,
            },
            {
                "task_id": "task_2",
                "action": "capture_keyframe",
                "target_node": "room701",
                "status": "pending",
                "requires_preflight": False,
            },
            {
                "task_id": "task_3",
                "action": "report",
                "message": "queued",
                "status": "pending",
                "requires_preflight": False,
            },
        ],
        "communication_policy": {
            "mode": "normal",
            "send": ["task_state", "navigation_feedback", "world_state_summary"],
            "drop": [],
            "reason": "normal link",
        },
    }


class TaskQueueTests(unittest.TestCase):
    def test_valid_queue_passes(self) -> None:
        validate_task_queue(make_queue())

    def test_rejects_invalid_action(self) -> None:
        queue = make_queue()
        queue["steps"][0]["action"] = "raw_api"

        with self.assertRaises(ValueError):
            validate_task_queue(queue)

    def test_rejects_targets_without_navigation(self) -> None:
        queue = make_queue()
        queue["steps"] = [queue["steps"][-1]]

        with self.assertRaises(ValueError):
            validate_task_queue(queue)

    def test_accepts_legacy_step_id_during_migration(self) -> None:
        queue = make_queue()
        queue["steps"][0]["step_id"] = queue["steps"][0].pop("task_id")

        validate_task_queue(queue)


if __name__ == "__main__":
    unittest.main()
