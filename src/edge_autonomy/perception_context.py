from __future__ import annotations

import json
import math
import time
import uuid
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Optional


MAX_SUMMARY_BYTES = 256 * 1024
STATUS_VALUES = {"fresh", "stale", "offline", "invalid", "uncalibrated"}
ProcessProbe = Callable[[int, str], bool]


class SensorSequenceTracker:
    """Reject sequence rollback within one stable producer instance."""

    def __init__(self) -> None:
        self._last: dict[str, tuple[Optional[str], int]] = {}

    def apply(self, envelopes: Iterable[Mapping[str, Any]]) -> list[dict[str, Any]]:
        checked: list[dict[str, Any]] = []
        for source in envelopes:
            envelope = dict(source)
            source_id = envelope.get("source_id")
            sequence = envelope.get("sequence")
            producer_instance = envelope.get("producer_instance_id")
            if not isinstance(producer_instance, str) or not producer_instance:
                producer_instance = None
            if isinstance(source_id, str) and isinstance(sequence, int) and not isinstance(sequence, bool):
                previous = self._last.get(source_id)
                if previous is not None and previous[0] == producer_instance and sequence < previous[1]:
                    envelope["status"] = "invalid"
                    reasons = list(envelope.get("status_reasons") or [])
                    reasons.append("sequence_rollback")
                    envelope["status_reasons"] = list(dict.fromkeys(reasons))[:32]
                elif envelope.get("status") in {"fresh", "stale"}:
                    self._last[source_id] = (producer_instance, sequence)
            checked.append(envelope)
        return checked


def now_ms() -> int:
    return int(time.time() * 1000)


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _integer(value: Any) -> Optional[int]:
    return int(value) if isinstance(value, int) and not isinstance(value, bool) else None


def _confidence(value: Any) -> Optional[float]:
    if not _is_number(value):
        return None
    number = float(value)
    return number if 0.0 <= number <= 1.0 else None


def _default_process_probe(pid: int, expected_process: str) -> bool:
    if pid <= 0 or not expected_process:
        return False
    try:
        cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", errors="replace")
    except OSError:
        return False
    return expected_process in cmdline


def _live_process_instance_id(pid: int) -> str:
    try:
        stat_fields = Path(f"/proc/{pid}/stat").read_text(encoding="ascii").split()
        process_start_ticks = int(stat_fields[21])
        boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
        if boot_id:
            return f"{boot_id}:{pid}:{process_start_ticks}"
    except (OSError, UnicodeError, ValueError, IndexError):
        pass
    return f"pid:{pid}"


def _read_json_object(path: Path) -> tuple[Optional[dict[str, Any]], Optional[str]]:
    try:
        size = path.stat().st_size
    except FileNotFoundError:
        return None, "artifact_missing"
    except OSError:
        return None, "artifact_unreadable"
    if size < 0 or size > MAX_SUMMARY_BYTES:
        return None, "artifact_too_large"
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None, "artifact_invalid_json"
    if not isinstance(value, dict):
        return None, "artifact_root_not_object"
    return value, None


def _read_error_status(error: Optional[str]) -> str:
    return "offline" if error in {"artifact_missing", "artifact_unreadable"} else "invalid"


def _base_envelope(
    *,
    source_id: str,
    source_kind: str,
    producer: str,
    received_ms: int,
    stale_ms: int,
    timestamp_ms: Optional[int] = None,
    sequence: Optional[int] = None,
    frame_id: Optional[str] = None,
    age_ms: Optional[int] = None,
    status: str = "offline",
    confidence: float = 0.0,
    calibration_status: str = "unknown",
    calibration_id: Optional[str] = None,
    producer_instance_id: Optional[str] = None,
    reasons: Iterable[str] = (),
    payload: Optional[Mapping[str, Any]] = None,
) -> dict[str, Any]:
    if status not in STATUS_VALUES:
        raise ValueError(f"unsupported sensor status: {status}")
    return {
        "schema_version": 1,
        "schema": "go2w_sensor_envelope_v1",
        "source_id": source_id,
        "source_kind": source_kind,
        "timestamp_ms": timestamp_ms,
        "received_ms": received_ms,
        "sequence": sequence,
        "age_ms": age_ms,
        "stale_ms": max(1, stale_ms),
        "frame_id": frame_id,
        "status": status,
        "confidence": confidence,
        "calibration_status": calibration_status,
        "calibration_id": calibration_id,
        "producer": producer,
        "producer_instance_id": producer_instance_id,
        "status_reasons": list(dict.fromkeys(str(reason) for reason in reasons if reason))[:32],
        "payload": dict(payload or {}),
    }


def _age(received_ms: int, timestamp_ms: Optional[int], latency_ms: Any = None) -> Optional[int]:
    if timestamp_ms is None:
        return None
    latency = int(math.ceil(float(latency_ms))) if _is_number(latency_ms) and float(latency_ms) > 0 else 0
    return received_ms - timestamp_ms + latency


def _pid_file_process(path: Optional[Path], expected: str, probe: ProcessProbe) -> Optional[int]:
    if path is None:
        return None
    try:
        text = path.read_text(encoding="ascii").strip()
        pid = int(text)
    except (OSError, UnicodeError, ValueError):
        return None
    return pid if probe(pid, expected) else None


def load_xt16_geometry_envelope(
    path: Path,
    *,
    received_ms: Optional[int] = None,
    stale_ms: int = 1000,
    pid_file: Optional[Path] = None,
    process_probe: ProcessProbe = _default_process_probe,
) -> dict[str, Any]:
    received = received_ms or now_ms()
    data, read_error = _read_json_object(path)
    if data is None:
        return _base_envelope(
            source_id="xt16_geometry",
            source_kind="lidar_geometry",
            producer="xt16_geometry_sidecar",
            received_ms=received,
            stale_ms=stale_ms,
            status=_read_error_status(read_error),
            reasons=[read_error or "artifact_unavailable"],
        )

    timestamp = _integer(data.get("timestamp_ms"))
    sequence = _integer(data.get("sequence"))
    frame_id = data.get("frame_id") if isinstance(data.get("frame_id"), str) and data.get("frame_id") else None
    confidence = _confidence(data.get("confidence"))
    parameters = data.get("parameters") if isinstance(data.get("parameters"), dict) else {}
    calibrated = parameters.get("calibrated") is True
    calibration_id = parameters.get("calibration_id")
    if not isinstance(calibration_id, str) or not calibration_id:
        calibration_id = None
    age = _age(received, timestamp, data.get("latency_ms"))
    roi_confidence = data.get("roi_confidence")
    clearances_valid = all(
        _is_number(data.get(name)) and float(data[name]) >= 0
        for name in ("front_clearance_m", "left_clearance_m", "right_clearance_m", "rear_clearance_m")
    )
    roi_valid = isinstance(roi_confidence, dict) and all(
        _confidence(roi_confidence.get(name)) is not None for name in ("front", "left", "right", "rear")
    )
    latency_valid = data.get("latency_ms") is None or (
        _is_number(data.get("latency_ms")) and float(data["latency_ms"]) >= 0
    )
    stale_reasons_valid = isinstance(data.get("stale_reasons"), list) and all(
        isinstance(item, str) for item in data["stale_reasons"]
    )
    reasons: list[str] = []
    invalid = (
        data.get("type") != "local_obstacle_summary"
        or data.get("schema_version") != 2
        or data.get("source") != "lidar_pointcloud"
        or timestamp is None
        or timestamp <= 0
        or sequence is None
        or sequence < 0
        or confidence is None
        or frame_id is None
        or not clearances_valid
        or not roi_valid
        or not isinstance(data.get("stale"), bool)
        or not stale_reasons_valid
        or not latency_valid
    )
    if invalid:
        reasons.append("invalid_xt16_contract")
    if age is not None and age < 0:
        invalid = True
        reasons.append("timestamp_in_future")
    producer_pid = _pid_file_process(pid_file, "xt16_lidar_geometry_summary.py", process_probe)
    if producer_pid is None:
        reasons.append("producer_offline")
    if data.get("stale") is True:
        reasons.extend(str(item) for item in data.get("stale_reasons", []) if isinstance(item, str))
        reasons.append("producer_declared_stale")
    if age is not None and age > stale_ms:
        reasons.append("stale_budget_exceeded")
    if not calibrated or calibration_id is None:
        reasons.append("xt16_calibration_unverified")

    if invalid:
        status = "invalid"
    elif producer_pid is None:
        status = "offline"
    elif not calibrated or calibration_id is None:
        status = "uncalibrated"
    elif data.get("stale") is True or age is None or age > stale_ms:
        status = "stale"
    else:
        status = "fresh"
    payload = {
        key: data.get(key)
        for key in (
            "front_clearance_m",
            "left_clearance_m",
            "right_clearance_m",
            "rear_clearance_m",
            "body_clearance_m",
            "low_hazard_clearance_m",
            "low_hazard_directions",
            "blocked_directions",
            "narrow_passage",
            "recommended_action",
            "roi_confidence",
        )
        if key in data
    }
    return _base_envelope(
        source_id="xt16_geometry",
        source_kind="lidar_geometry",
        producer="xt16_geometry_sidecar",
        received_ms=received,
        stale_ms=stale_ms,
        timestamp_ms=timestamp if timestamp and timestamp > 0 else None,
        sequence=sequence if sequence is not None and sequence >= 0 else None,
        frame_id=frame_id,
        age_ms=age,
        status=status,
        confidence=confidence or 0.0,
        calibration_status="verified" if calibrated and calibration_id else "pending",
        calibration_id=calibration_id,
        producer_instance_id=_live_process_instance_id(producer_pid) if producer_pid is not None else None,
        reasons=reasons,
        payload=payload,
    )


def _d435_owner_online(data: Mapping[str, Any], probe: ProcessProbe) -> bool:
    owner = data.get("owner") if isinstance(data.get("owner"), dict) else {}
    pid = _integer(owner.get("pid"))
    expected = owner.get("expected_process")
    declared_online = bool(
        pid
        and isinstance(expected, str)
        and expected
        and owner.get("running") is True
        and owner.get("status") == "owned"
        and probe(pid, expected)
    )
    if not declared_online:
        return False
    if probe is not _default_process_probe:
        return True
    return _d435_producer_instance(data) == _live_process_instance_id(pid)


def _d435_producer_instance(data: Mapping[str, Any]) -> Optional[str]:
    owner = data.get("owner") if isinstance(data.get("owner"), dict) else {}
    pid = _integer(owner.get("pid"))
    process_start_ticks = _integer(owner.get("process_start_ticks"))
    boot_id = owner.get("boot_id")
    if pid is None or process_start_ticks is None or not isinstance(boot_id, str) or not boot_id:
        return None
    return f"{boot_id}:{pid}:{process_start_ticks}"


def _d435_child_envelope(
    data: Mapping[str, Any],
    child_name: str,
    *,
    received_ms: int,
    stale_ms: int,
    online: bool,
    producer_instance_id: Optional[str],
) -> dict[str, Any]:
    child = data.get(child_name) if isinstance(data.get(child_name), dict) else {}
    timestamp = _integer(child.get("captured_at_ms") or child.get("timestamp_ms"))
    sequence = _integer(child.get("frame_sequence"))
    frame_id = child.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id:
        frame_id = data.get("capture", {}).get("frame_id") if isinstance(data.get("capture"), dict) else None
    confidence = _confidence(child.get("confidence"))
    if child_name == "yolo" and confidence is None:
        objects = child.get("objects") if isinstance(child.get("objects"), list) else []
        object_confidences = [
            value
            for item in objects
            if isinstance(item, dict)
            for value in [_confidence(item.get("confidence"))]
            if value is not None
        ]
        confidence = max(object_confidences, default=_confidence(data.get("confidence")) or 0.0)
    age = _age(received_ms, timestamp)
    source_status = child.get("source_status")
    if child_name == "yolo":
        fresh_values = {"fresh", "depth_insufficient"}
        stale_values = {"event_only_idle", "stale", "clock_skew"}
        offline_values = {"unavailable", "offline"}
    else:
        fresh_values = {"fresh"}
        stale_values = {"stale"}
        offline_values = {"offline"}
    reasons: list[str] = []
    generation_matches = (
        isinstance(child.get("generation_id"), str)
        and child.get("generation_id") == data.get("generation_id")
    )
    claimed_data = source_status in fresh_values | stale_values
    invalid = claimed_data and (
        timestamp is None
        or timestamp <= 0
        or sequence is None
        or sequence <= 0
        or confidence is None
        or not generation_matches
    )
    if child_name == "depth" and claimed_data:
        roi = child.get("roi_confidence")
        invalid = invalid or (
            not _is_number(child.get("front_clearance_m"))
            or float(child["front_clearance_m"]) < 0
            or not isinstance(roi, dict)
            or _confidence(roi.get("front")) is None
        )
    if invalid:
        reasons.append(f"invalid_d435_{child_name}_contract")
    if age is not None and age < 0:
        invalid = True
        reasons.append("timestamp_in_future")
    if not online:
        reasons.append("producer_offline")
    if source_status in stale_values or child.get("stale") is True:
        reasons.append("producer_declared_stale")
    if source_status not in fresh_values | stale_values | offline_values:
        reasons.append("producer_status_not_fresh")
    if age is not None and age > stale_ms:
        reasons.append("stale_budget_exceeded")

    if source_status in offline_values:
        status = "offline"
    elif invalid:
        status = "invalid"
    elif not online:
        status = "offline"
    elif source_status in stale_values or child.get("stale") is True or age is None or age > stale_ms:
        status = "stale"
    elif source_status not in fresh_values:
        status = "offline"
    else:
        status = "fresh"
    if child_name == "depth":
        payload_keys = (
            "front_clearance_m",
            "left_clearance_m",
            "right_clearance_m",
            "center_distance_m",
            "roi_confidence",
            "generation_id",
        )
        source_id = "d435_depth"
        source_kind = "rgbd_depth_geometry"
    else:
        payload_keys = ("objects", "events", "generation_id", "capture_frame_lag", "source_status")
        source_id = "d435_yolo"
        source_kind = "visual_object_semantics"
    payload = {key: child.get(key) for key in payload_keys if key in child}
    return _base_envelope(
        source_id=source_id,
        source_kind=source_kind,
        producer="d435_perception_sidecar",
        received_ms=received_ms,
        stale_ms=stale_ms,
        timestamp_ms=timestamp if timestamp and timestamp > 0 else None,
        sequence=sequence if sequence is not None and sequence >= 0 else None,
        frame_id=frame_id if isinstance(frame_id, str) and frame_id else None,
        age_ms=age,
        status=status,
        confidence=confidence or 0.0,
        calibration_status="unknown",
        calibration_id=None,
        producer_instance_id=producer_instance_id,
        reasons=reasons,
        payload=payload,
    )


def load_d435_envelopes(
    path: Path,
    *,
    received_ms: Optional[int] = None,
    depth_stale_ms: int = 1000,
    yolo_stale_ms: int = 3000,
    process_probe: ProcessProbe = _default_process_probe,
) -> list[dict[str, Any]]:
    received = received_ms or now_ms()
    data, read_error = _read_json_object(path)
    if data is None:
        return [
            _base_envelope(
                source_id=source_id,
                source_kind=source_kind,
                producer="d435_perception_sidecar",
                received_ms=received,
                stale_ms=stale_ms,
                status=_read_error_status(read_error),
                reasons=[read_error or "artifact_unavailable"],
            )
            for source_id, source_kind, stale_ms in (
                ("d435_depth", "rgbd_depth_geometry", depth_stale_ms),
                ("d435_yolo", "visual_object_semantics", yolo_stale_ms),
            )
        ]
    identity_valid = (
        data.get("schema_version") == 1
        and data.get("schema") == "go2w_d435_perception_summary_v1"
        and data.get("source") == "d435_perception"
        and data.get("producer") == "d435_perception_sidecar"
        and isinstance(data.get("generation_id"), str)
        and bool(data.get("generation_id"))
    )
    online = _d435_owner_online(data, process_probe)
    producer_instance_id = _d435_producer_instance(data)
    if producer_instance_id is None:
        identity_valid = False
    envelopes = [
        _d435_child_envelope(
            data,
            "depth",
            received_ms=received,
            stale_ms=depth_stale_ms,
            online=online,
            producer_instance_id=producer_instance_id,
        ),
        _d435_child_envelope(
            data,
            "yolo",
            received_ms=received,
            stale_ms=yolo_stale_ms,
            online=online,
            producer_instance_id=producer_instance_id,
        ),
    ]
    if not identity_valid:
        for envelope in envelopes:
            envelope["status"] = "invalid"
            envelope["status_reasons"].insert(0, "invalid_d435_summary_identity")
    return envelopes


def load_ti_nx_envelope(
    path: Path,
    *,
    received_ms: Optional[int] = None,
    stale_ms: int = 3000,
    producer_instance_id: Optional[str] = None,
) -> dict[str, Any]:
    received = received_ms or now_ms()
    data, read_error = _read_json_object(path)
    if data is None:
        return _base_envelope(
            source_id="ti_nx_radar",
            source_kind="radar_semantics",
            producer="nx_edge_bridge",
            received_ms=received,
            stale_ms=stale_ms,
            status=_read_error_status(read_error),
            reasons=[read_error or "artifact_unavailable"],
        )
    timestamp = _integer(data.get("timestamp_ms"))
    sequence = _integer(data.get("sequence"))
    confidence = _confidence(data.get("confidence"))
    node_id = data.get("node_id")
    source = data.get("source")
    health = data.get("health") if isinstance(data.get("health"), dict) else {}
    policy = data.get("policy") if isinstance(data.get("policy"), dict) else {}
    age = _age(received, timestamp, data.get("latency_ms"))
    reasons: list[str] = []
    invalid = (
        data.get("schema_version") != 1
        or not isinstance(node_id, str)
        or not node_id
        or not isinstance(data.get("sensor_type"), str)
        or not data.get("sensor_type")
        or not isinstance(source, str)
        or not source
        or timestamp is None
        or timestamp <= 0
        or sequence is None
        or sequence < 0
        or confidence is None
        or data.get("clock_domain") != "unix_epoch_ms"
    )
    if invalid:
        reasons.append("invalid_ti_nx_contract")
    if age is not None and age < 0:
        invalid = True
        reasons.append("timestamp_in_future")
    if not isinstance(producer_instance_id, str) or not producer_instance_id:
        reasons.append("producer_online_unverified")
    if data.get("stale") is True or health.get("status") != "ok":
        reasons.append("producer_declared_stale_or_unhealthy")
    if age is not None and age > stale_ms:
        reasons.append("stale_budget_exceeded")
    calibrated = policy.get("calibrated") is True
    calibration_id = policy.get("calibration_id")
    if not isinstance(calibration_id, str) or not calibration_id:
        calibration_id = None

    if invalid:
        status = "invalid"
    elif not isinstance(producer_instance_id, str) or not producer_instance_id:
        status = "offline"
    elif data.get("stale") is True or health.get("status") != "ok" or age is None or age > stale_ms:
        status = "stale"
    else:
        status = "fresh"
    payload = {
        "node_id": node_id,
        "sensor_type": data.get("sensor_type"),
        "observations": data.get("observations", [])[:32] if isinstance(data.get("observations"), list) else [],
        "events": data.get("events", [])[:32] if isinstance(data.get("events"), list) else [],
        "summary": data.get("summary") if isinstance(data.get("summary"), dict) else {},
        "policy": {
            "mode": policy.get("mode", "semantic_only"),
            "safety_candidate": policy.get("safety_candidate") is True,
            "safety_wired": False,
        },
    }
    return _base_envelope(
        source_id=f"ti_nx:{node_id}" if isinstance(node_id, str) and node_id else "ti_nx_radar",
        source_kind="radar_semantics",
        producer="nx_edge_bridge",
        received_ms=received,
        stale_ms=stale_ms,
        timestamp_ms=timestamp if timestamp and timestamp > 0 else None,
        sequence=sequence if sequence is not None and sequence >= 0 else None,
        frame_id=None,
        age_ms=age,
        status=status,
        confidence=confidence or 0.0,
        calibration_status="verified" if calibrated and calibration_id else "unknown",
        calibration_id=calibration_id,
        producer_instance_id=producer_instance_id if isinstance(producer_instance_id, str) and producer_instance_id else None,
        reasons=reasons,
        payload=payload,
    )


def reserved_motion_envelope(
    source_id: str,
    source_kind: str,
    *,
    received_ms: Optional[int] = None,
) -> dict[str, Any]:
    if source_kind not in {"imu_motion", "odometry_motion"}:
        raise ValueError("reserved motion source_kind must be imu_motion or odometry_motion")
    return _base_envelope(
        source_id=source_id,
        source_kind=source_kind,
        producer="motion_summary_adapter",
        received_ms=received_ms or now_ms(),
        stale_ms=1000,
        calibration_status="unknown",
        reasons=["adapter_not_implemented"],
    )


def _attributed(items: Any, envelope: Mapping[str, Any], limit: int = 64) -> list[dict[str, Any]]:
    if not isinstance(items, list) or envelope.get("status") != "fresh":
        return []
    source_id = str(envelope["source_id"])
    timestamp = envelope.get("timestamp_ms")
    if not isinstance(timestamp, int):
        return []
    result = []
    for item in items[:limit]:
        if isinstance(item, dict):
            result.append({**item, "source_id": source_id, "timestamp_ms": timestamp})
    return result


_ENVELOPE_FIELDS = {
    "schema_version",
    "schema",
    "source_id",
    "source_kind",
    "timestamp_ms",
    "received_ms",
    "sequence",
    "age_ms",
    "stale_ms",
    "frame_id",
    "status",
    "confidence",
    "calibration_status",
    "calibration_id",
    "producer",
    "producer_instance_id",
    "status_reasons",
    "payload",
}


def _validate_and_refresh_envelope(source: Mapping[str, Any], generated_at_ms: int) -> dict[str, Any]:
    envelope = dict(source)
    missing = sorted(_ENVELOPE_FIELDS - set(envelope))
    if missing:
        raise ValueError("sensor envelope missing fields: " + ",".join(missing))
    if envelope.get("schema_version") != 1 or envelope.get("schema") != "go2w_sensor_envelope_v1":
        raise ValueError("invalid sensor envelope identity")
    if not isinstance(envelope.get("source_id"), str) or not envelope["source_id"]:
        raise ValueError("every source must have a non-empty source_id")
    if not isinstance(envelope.get("source_kind"), str) or not envelope["source_kind"]:
        raise ValueError("every source must have a non-empty source_kind")
    if not isinstance(envelope.get("producer"), str) or not envelope["producer"]:
        raise ValueError("every source must have a non-empty producer")
    if envelope.get("status") not in STATUS_VALUES:
        raise ValueError("invalid sensor envelope status")
    if _confidence(envelope.get("confidence")) is None:
        raise ValueError("sensor envelope confidence must be between 0 and 1")
    if not isinstance(envelope.get("payload"), dict):
        raise ValueError("sensor envelope payload must be an object")
    if not isinstance(envelope.get("status_reasons"), list) or any(
        not isinstance(reason, str) or not reason for reason in envelope["status_reasons"]
    ):
        raise ValueError("sensor envelope status_reasons must be strings")
    received = _integer(envelope.get("received_ms"))
    stale_ms = _integer(envelope.get("stale_ms"))
    if received is None or received <= 0 or stale_ms is None or stale_ms <= 0:
        raise ValueError("sensor envelope received_ms and stale_ms must be positive integers")

    status = str(envelope["status"])
    age = _integer(envelope.get("age_ms"))
    timestamp = _integer(envelope.get("timestamp_ms"))
    sequence = _integer(envelope.get("sequence"))
    producer_instance = envelope.get("producer_instance_id")
    if status == "fresh" and (
        age is None
        or age < 0
        or timestamp is None
        or timestamp <= 0
        or sequence is None
        or sequence < 0
        or not isinstance(producer_instance, str)
        or not producer_instance
    ):
        raise ValueError("fresh sensor envelope lacks timestamp, sequence, age, or producer instance")
    if age is not None and status not in {"offline", "invalid"}:
        elapsed = generated_at_ms - received
        if elapsed < 0:
            envelope["status"] = "invalid"
            envelope["status_reasons"] = list(
                dict.fromkeys([*envelope["status_reasons"], "context_generated_before_received"])
            )[:32]
        else:
            effective_age = age + elapsed
            envelope["age_ms"] = effective_age
            if effective_age < 0:
                envelope["status"] = "invalid"
                envelope["status_reasons"] = list(
                    dict.fromkeys([*envelope["status_reasons"], "timestamp_in_future"])
                )[:32]
            elif effective_age > stale_ms and envelope["status"] in {"fresh", "uncalibrated"}:
                envelope["status"] = "stale"
                envelope["status_reasons"] = list(
                    dict.fromkeys([*envelope["status_reasons"], "context_stale_budget_exceeded"])
                )[:32]
    return envelope


def build_perception_context(
    envelopes: Iterable[Mapping[str, Any]],
    *,
    generated_at_ms: Optional[int] = None,
    context_id: Optional[str] = None,
    sequence_tracker: Optional[SensorSequenceTracker] = None,
) -> dict[str, Any]:
    generated = generated_at_ms if generated_at_ms is not None else now_ms()
    if generated <= 0:
        raise ValueError("generated_at_ms must be positive")
    sources = [_validate_and_refresh_envelope(envelope, generated) for envelope in envelopes]
    if len(sources) > 32:
        raise ValueError("PerceptionContext supports at most 32 sources")
    if sequence_tracker is not None:
        sources = sequence_tracker.apply(sources)
    source_ids = [source.get("source_id") for source in sources]
    if any(not isinstance(source_id, str) or not source_id for source_id in source_ids):
        raise ValueError("every source must have a non-empty source_id")
    if len(source_ids) != len(set(source_ids)):
        raise ValueError("source_id values must be unique")

    by_id = {str(source["source_id"]): source for source in sources}
    xt16 = by_id.get("xt16_geometry")
    depth = by_id.get("d435_depth")
    yolo = by_id.get("d435_yolo")
    radar_sources = [source for source in sources if source.get("source_kind") == "radar_semantics"]
    motion_sources = [
        source
        for source in sources
        if source.get("source_kind") in {"imu_motion", "odometry_motion"} and source.get("status") == "fresh"
    ]
    primary_geometry = None
    if (
        xt16
        and xt16.get("status") == "fresh"
        and xt16.get("calibration_status") == "verified"
    ):
        primary_geometry = {
            **dict(xt16.get("payload") or {}),
            "source_id": xt16["source_id"],
            "timestamp_ms": xt16["timestamp_ms"],
        }
    forward_supplements = []
    if depth and depth.get("status") == "fresh":
        forward_supplements.append(
            {
                **dict(depth.get("payload") or {}),
                "source_id": depth["source_id"],
                "timestamp_ms": depth["timestamp_ms"],
            }
        )
    local_geometry = {
        "primary": primary_geometry,
        "forward_supplements": forward_supplements,
    }
    robot_motion = {
        "sources": [
            {
                **dict(source.get("payload") or {}),
                "source_id": source["source_id"],
                "timestamp_ms": source["timestamp_ms"],
            }
            for source in motion_sources
        ]
    }
    visual_objects = _attributed((yolo or {}).get("payload", {}).get("objects"), yolo or {})
    radar_tracks: list[dict[str, Any]] = []
    risk_events: list[dict[str, Any]] = []
    for source in radar_sources:
        payload = source.get("payload") if isinstance(source.get("payload"), dict) else {}
        radar_tracks.extend(_attributed(payload.get("observations"), source))
        risk_events.extend(_attributed(payload.get("events"), source))
    degraded = sorted(
        str(source["source_id"])
        for source in sources
        if source.get("status") != "fresh"
    )
    return {
        "schema_version": 1,
        "schema": "go2w_perception_context_v1",
        "context_id": context_id or f"pc-{generated}-{uuid.uuid4().hex[:8]}",
        "generated_at_ms": generated,
        "robot_motion": robot_motion,
        "local_geometry": local_geometry,
        "visual_objects": visual_objects[:64],
        "radar_tracks": radar_tracks[:64],
        "risk_events": risk_events[:64],
        "sources": sources,
        "degraded_capabilities": degraded,
        "policy": {
            "motion_authority": "slam_gateway",
            "llm_direct_motion": False,
            "raw_sensor_streams_allowed": False,
            "execution_chain": [
                "task_queue",
                "mission_decision_engine",
                "slam_gateway",
                "unitree_sdk",
            ],
        },
    }
