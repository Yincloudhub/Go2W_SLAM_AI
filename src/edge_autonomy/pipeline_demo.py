from __future__ import annotations

import json
import time

from .models import NavigationGoal, Pose2D, RiskEvent, RobotState, SemanticObject, WorldState
from .safety import LinkQuality, SafetySupervisor
from .slam_adapter import InMemorySlamAdapter


def build_demo_world_state() -> WorldState:
    return WorldState(
        timestamp_ms=int(time.time() * 1000),
        frame_id="map",
        robot=RobotState(pose=Pose2D(0.0, 0.0, 0.0), mode="autonomy", battery_percent=86.0),
        objects=[
            SemanticObject(
                object_id="person-1",
                category="person",
                pose=Pose2D(1.2, 0.3, 0.0),
                distance_m=1.24,
                risk_level="high",
                traversable=False,
                is_dynamic=True,
                velocity_mps=0.4,
            )
        ],
        risk_events=[RiskEvent(event_type="human_near_path", severity="high", description="person entered path corridor", distance_m=1.24)],
        task_phase="navigate_to_checkpoint",
    )


def main() -> None:
    adapter = InMemorySlamAdapter()
    supervisor = SafetySupervisor()

    goal = NavigationGoal(goal_id="checkpoint-1", target_pose=Pose2D(2.0, 0.0, 0.0))
    adapter.submit_navigation_goal(goal)

    world_state = build_demo_world_state()
    feedback = adapter.get_feedback()
    decision = supervisor.evaluate(
        world_state=world_state,
        navigation_feedback=feedback,
        link_quality=LinkQuality(bandwidth_kbps=128.0, latency_ms=620.0),
    )

    print(json.dumps({"world_state": world_state.to_dict(), "feedback": feedback.__dict__, "decision": decision.__dict__}, ensure_ascii=False, indent=2, default=str))


if __name__ == "__main__":
    main()
