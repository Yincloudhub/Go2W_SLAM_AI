# Stereo Depth Camera Integration

This note defines how to add a binocular/stereo depth camera without increasing latency in the GO2W closed-loop runtime.

## Current Performance Position

The closed-loop path already has the main protections needed for real-time behavior:

- C++ `SafetyGate` and `QueueExecutor` own the deterministic execution path.
- SLAM polling, UI refresh, operator feedback, and LLM feedback use separate intervals.
- `arrival_samples`, `operator_feedback`, and `llm_feedback_*` are bounded buffers.
- Live LLM calls are kept out of high-frequency progress loops by default.
- Runtime reports expose `poll_overruns`, `max_loop_elapsed_s`, and dropped event counts.

This is architecturally safe, but the real performance number still needs an on-robot benchmark with SLAM and LiDAR running. The most useful field metric is:

```text
poll_overruns == 0 during normal navigation
max_loop_elapsed_s < slam_poll_interval_s
gateway failures below gateway_error_limit
```

## Integration Boundary

Do not pass raw stereo images, full depth maps, or dense camera point clouds into the LLM, UI, or queue executor. The camera process should publish only a compact depth summary:

```json
{
  "source": "stereo_depth",
  "timestamp_ms": 0,
  "frame_id": "camera_depth_optical_frame",
  "front_clearance_m": 1.2,
  "left_clearance_m": 1.8,
  "right_clearance_m": 1.4,
  "rear_clearance_m": null,
  "confidence": 0.82,
  "latency_ms": 70,
  "stale": false
}
```

`src/edge_autonomy/perception_fusion.py` adds `DepthCameraSummary` and `fuse_local_obstacle_summary()`. Stereo depth may reduce the clearances seen by the safety layer, but it must never increase LiDAR clearance. This keeps the fusion conservative.

## Recommended Frequencies

| Path | Rate | Notes |
| --- | ---: | --- |
| Camera driver | Native or 15-30 Hz | Hardware/driver thread only. |
| Depth ROI summary | 5-10 Hz | Crop to front/side regions, downsample, compute min/percentile clearance. |
| Safety fusion | Same as SLAM poll, usually 1 Hz now | Consume latest valid summary only. |
| UI display | 1-2 Hz | Show source/confidence/latency, not frames. |
| LLM feedback | Terminal events or <= 0.2 Hz | Summarize state changes, never stream images. |

## Performance Rules

1. Run camera perception as an optional process on the robot, not on the Windows UI machine.
2. Use hardware depth from the stereo camera when available. Avoid neural depth in the control loop.
3. Downsample depth and compute region-of-interest clearances instead of processing full frames in Python.
4. Use a bounded queue or latest-value cache between camera perception and the main runtime.
5. If `latency_ms` or sample age exceeds 300-500 ms, mark the camera summary stale.
6. If confidence is below threshold, ignore the camera summary for blocking decisions.
7. Camera absence must not prevent SLAM startup, localization, dry-run planning, or LiDAR-only navigation.
8. Do not log raw frames by default; keep only short ring buffers for debugging.

## Failure Strategy

- LiDAR valid, camera stale: continue LiDAR-only.
- LiDAR clear, camera near obstacle with confidence: slow or pause conservatively.
- LiDAR blocked, camera clear: stay blocked; stereo cannot relax the LiDAR safety decision.
- Camera process crash: mark source unavailable, keep queue executor alive.
- Camera timestamp jumps backward or frames stall: mark stale and expose this in UI diagnostics.

## Next Wiring Step

The next implementation step is to add a small robot-side publisher that turns the chosen camera topic into `DepthCameraSummary` JSON, then let the operator core read the latest summary before each `SafetyGate` runtime check. The C++ hot loop should read the latest compact value only; it should never wait for camera processing to finish.
