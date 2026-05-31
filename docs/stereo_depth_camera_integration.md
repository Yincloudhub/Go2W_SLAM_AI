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
  "center_distance_m": 1.1,
  "center_window_m": 1.0,
  "front_clearance_m": 1.2,
  "left_clearance_m": 1.8,
  "right_clearance_m": 1.4,
  "rear_clearance_m": null,
  "confidence": 0.82,
  "roi_confidence": {
    "front": 0.76,
    "left": 0.91,
    "right": 0.88,
    "center_window": 0.8
  },
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
| UI display | 1-2 Hz | Show source/confidence/latency, not frames. The Web panel treats camera summaries older than `GO2W_STEREO_STALE_MS` as stale; default is 5000 ms so a 1-2 Hz UI does not falsely flap on one missed refresh. |
| LLM feedback | Terminal events or <= 0.2 Hz | Summarize state changes, never stream images. |

## Performance Rules

1. Run camera perception as an optional process on the robot, not on the Windows UI machine.
2. Use hardware depth from the stereo camera when available. Avoid neural depth in the control loop.
3. Downsample depth and compute region-of-interest clearances instead of processing full frames in Python.
4. Use a bounded queue or latest-value cache between camera perception and the main runtime.
5. If `latency_ms` or sample age exceeds 300-500 ms, the safety-fusion layer should mark the camera summary stale; the UI may use a looser display-only threshold such as 5000 ms.
6. Treat `confidence` as whole-image valid-depth ratio, not as a center-distance
   validity flag. Use `center_distance_m`/`center_window_m` for center-point UI
   feedback, and `roi_confidence.front` with `front_clearance_m` for conservative
   safety fusion.
7. If the relevant ROI confidence is below threshold, ignore that ROI for
   blocking decisions.
8. Camera absence must not prevent SLAM startup, localization, dry-run planning, or LiDAR-only navigation.
9. Do not log raw frames by default; keep only short ring buffers for debugging.

## Failure Strategy

- LiDAR valid, camera stale: continue LiDAR-only.
- LiDAR clear, camera near obstacle with confidence: slow or pause conservatively.
- LiDAR blocked, camera clear: stay blocked; stereo cannot relax the LiDAR safety decision.
- Camera process crash: mark source unavailable, keep queue executor alive.
- Camera timestamp jumps backward or frames stall: mark stale and expose this in UI diagnostics.

## Next Wiring Step

The next implementation step is to add a small robot-side publisher that turns the chosen camera topic into `DepthCameraSummary` JSON, then let the operator core read the latest summary before each `SafetyGate` runtime check. The C++ hot loop should read the latest compact value only; it should never wait for camera processing to finish.

## Robot-Side Summary Exporter

`scripts/realsense_depth_summary.py` is the first repo-owned bridge for the D435I
depth camera. It captures only a small number of depth frames, computes
left/front/right ROI clearances, and writes a compact JSON summary:

```bash
cd ~/go2w_slam_agent
python3 scripts/realsense_depth_summary.py \
  --output artifacts/stereo_depth_summary.json \
  --frames 3 \
  --fps 15 \
  --pretty
```

For recording/debug sessions, keep the camera publisher deliberately low-rate:

```bash
python3 scripts/realsense_depth_summary.py \
  --output artifacts/stereo_depth_summary.json \
  --loop-interval-s 1.0 \
  --max-samples 0
```

The loop keeps the RealSense pipeline open and atomically replaces the compact
JSON file on each update. It still does not send raw frames to the UI or LLM.

The script can also read a saved `depth_raw.npy` for offline validation:

```bash
python3 scripts/realsense_depth_summary.py \
  --from-npy /home/unitree/depthcamera/output/depth_raw.npy \
  --output artifacts/stereo_depth_summary.json \
  --pretty
```

The summary is diagnostic until C++ consumes it. A low confidence value should
be shown in the UI but ignored by safety fusion; it must not block the robot by
itself.

The operator Web UI reads this file through `--stereo-summary-path` and marks it
stale by `--stereo-stale-ms` / `GO2W_STEREO_STALE_MS` without subscribing to raw
camera streams. This keeps camera display decoupled from the real-time loop.

2026-05-31 prone-safe check on `unitree@192.168.123.18`:

- `rs-enumerate-devices -s` detected `Intel RealSense D435I`, serial
  `346222072418`, firmware `5.17.0.10`.
- `/home/unitree/depthcamera/capture_rs_safe.py` captured one color and depth
  frame at 640x480.
- The saved depth frame had whole-image valid-depth confidence around `0.337`.
  This does not mean the camera is unusable. It means many pixels are invalid in
  the current prone pose. The exact center pixel was invalid (`0 m`), while the
  wider center/front region still produced nearby valid depth around `0.24 m`.
  UI should display these values separately; safety fusion should use ROI
  confidence rather than the whole-image score alone.
