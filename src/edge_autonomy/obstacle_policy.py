from __future__ import annotations

from collections.abc import Mapping


FRONT_PAUSE_M = 0.80
FRONT_SLOW_M = 1.50
SIDE_PAUSE_M = 0.20
SIDE_SLOW_M = 0.60
REAR_PAUSE_M = 0.30
REAR_SLOW_M = 0.50
CONSERVATIVE_SPEED_MPS = 0.20


def _below(value: object, threshold: float) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and 0.0 <= float(value) < threshold
    )


def blocked_directions(clearance: Mapping[str, object]) -> list[str]:
    thresholds = {
        "front": FRONT_PAUSE_M,
        "left": SIDE_PAUSE_M,
        "right": SIDE_PAUSE_M,
        "rear": REAR_PAUSE_M,
    }
    return [
        direction
        for direction in ("front", "left", "right", "rear")
        if _below(clearance.get(direction), thresholds[direction])
    ]


def recommended_action(clearance: Mapping[str, object]) -> str:
    if (
        _below(clearance.get("front"), FRONT_PAUSE_M)
        or _below(clearance.get("left"), SIDE_PAUSE_M)
        or _below(clearance.get("right"), SIDE_PAUSE_M)
        or _below(clearance.get("rear"), REAR_PAUSE_M)
    ):
        return "pause"
    if (
        _below(clearance.get("front"), FRONT_SLOW_M)
        or _below(clearance.get("left"), SIDE_SLOW_M)
        or _below(clearance.get("right"), SIDE_SLOW_M)
        or _below(clearance.get("rear"), REAR_SLOW_M)
    ):
        return "go_slow"
    return "normal"


def narrow_passage(clearance: Mapping[str, object]) -> bool:
    return _below(clearance.get("left"), SIDE_SLOW_M) and _below(
        clearance.get("right"), SIDE_SLOW_M
    )
