from edge_autonomy.execution_report import summarize_agent_output


def test_summary_extracts_semantic_trace_target_fields() -> None:
    output = {
        "command": "去赵博办公室门口",
        "execute": False,
        "nav_speed_mps": 0.3,
        "nav_mode": 1,
        "steps": [
            {
                "step": "closed_loop",
                "result": {
                    "result": {
                        "semantic_trace": {
                            "target": {
                                "node_id": "zhao_bo_office_front",
                                "name": "赵博办公室门口",
                                "distance_from_robot_m": 2.4,
                                "needs_calibration": True,
                                "photo_required": True,
                            }
                        },
                        "planner": {
                            "llm_elapsed_s": 0.0,
                            "plan": {"mode": "mapped_navigation", "reason": "ok", "plan_id": "p1"},
                            "slam_command": {"target_node": "zhao_bo_office_front"},
                        },
                        "execution": {"executed": False, "blocked_reason": "dry run"},
                    }
                },
            }
        ],
    }

    summary = summarize_agent_output(output)

    assert summary["target_node"] == "zhao_bo_office_front"
    assert summary["target_name"] == "赵博办公室门口"
    assert summary["target_needs_calibration"] is True
    assert summary["target_photo_required"] is True
    assert summary["target_distance_from_robot_m"] == 2.4


def test_summary_reports_automatic_relocation_as_blocked() -> None:
    summary = summarize_agent_output(
        {
            "steps": [
                {
                    "step": "go_auto_relocate_blocked",
                    "reason": "supervised anchor relocation required",
                }
            ]
        }
    )

    assert summary["auto_relocated"] is False
    assert summary["relocation_required"] is True
    assert summary["relocation_reason"] == "supervised anchor relocation required"
