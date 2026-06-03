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
8. Camera absence must not prevent SLAM startup, localization, or dry-run planning. Until XT16 point-cloud geometry is wired into the gateway, it must block real navigation.
9. Do not log raw frames by default; keep only short ring buffers for debugging.

## Failure Strategy

- SLAM valid, camera stale: keep localization and dry-run planning online, but block real navigation until XT16 point-cloud geometry is wired.
- LiDAR clear, camera near obstacle with confidence: slow or pause conservatively.
- LiDAR blocked, camera clear: stay blocked; stereo cannot relax the LiDAR safety decision.
- Camera process crash: mark source unavailable, keep the runtime alive, and block new real navigation.
- Camera timestamp jumps backward or frames stall: mark stale and expose this in UI diagnostics.
- Detector process alive but source stays stale: keep the main LiDAR/SLAM loop
  running and recover only the optional sidecar with
  `bash scripts/go2w_deepyolo_sidecar.sh restart-if-stale`.

## Current Motion-Safety Wiring

`scripts/go2w_stereo_depth_sidecar.sh` now keeps the D435 ROI summary fresh at low rate. The robot gateway, Python preflight, C++ `SafetyGate`, and Web UI execute toggle consume only the latest compact value. They never wait for camera processing to finish and never read raw camera frames.

The summary is currently an additional hard gate because XT16 point-cloud geometry has not yet been wired into `LidarGeometryPerception`. A missing, stale, low-confidence, or near-obstacle D435 summary blocks real motion while still allowing SLAM startup, localization debugging, topology inspection, and dry-run planning.

This is an incident-response safety floor, not the final architecture. The formal next step remains implementing XT16 point-cloud-derived front/left/right clearance and then fusing D435 conservatively so either sensor can increase caution but neither can silently relax a LiDAR block.

## Robot-Side Summary Exporter

`scripts/realsense_depth_summary.py` is the first repo-owned bridge for the D435I
depth camera. It captures only a small number of depth frames, computes
left/front/right ROI clearances, and writes a compact JSON summary:

```bash
cd /home/unitree/Go2W_SLAM_AI
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

The summary is now consumed by the motion-safety chain. Safety checks use the
front/left/right ROI confidence values rather than treating whole-image valid
fraction as a direct pass/fail signal.

The operator Web UI reads this file through `--stereo-summary-path` and marks it
stale by `--stereo-stale-ms` / `GO2W_STEREO_STALE_MS` without subscribing to raw
camera streams. This keeps camera display decoupled from the real-time loop.

## DeepYOLO Semantic Bridge

The robot currently also has a TensorRT YOLO + RealSense prototype under:

```text
/home/unitree/librealsense/examples/DeepYolo_test
/home/unitree/DeepYolo
```

That prototype should stay an optional perception side process. The GO2W runtime
must not depend on its window display, TensorRT loop, raw color frames, raw depth
frames, or unbounded JSONL logs.

The repo-owned bridge is:

```bash
cd ~/Go2W_SLAM_AI
python3 scripts/deepyolo_semantic_bridge.py \
  --input-dir /home/unitree/librealsense/examples/DeepYolo_test/output \
  --output artifacts/vision_semantic_summary.json \
  --loop-interval-s 0.5 \
  --max-samples 0
```

or, as a wrapper:

```bash
bash scripts/start_go2w_deepyolo_bridge.sh
```

The original DeepYOLO example opens OpenCV GUI windows and can crash in SSH
headless sessions with a GTK initialization error. For robot-side service or
short detection checks, generate a headless binary without modifying the vendor
source:

```bash
cd ~/Go2W_SLAM_AI
bash scripts/build_deepyolo_headless.sh
GO2W_DEEPYOLO_MAX_FRAMES=30 \
  /home/unitree/librealsense/examples/DeepYolo_test/go2w_headless/build/yolo_test_realsense_headless
python3 scripts/deepyolo_semantic_bridge.py \
  --input-dir /home/unitree/librealsense/examples/DeepYolo_test/output \
  --output artifacts/vision_semantic_summary.json \
  --pretty
```

For a controlled longer-running check, use the optional sidecar manager:

```bash
bash scripts/go2w_deepyolo_sidecar.sh build
bash scripts/go2w_deepyolo_sidecar.sh start
bash scripts/go2w_deepyolo_sidecar.sh health
bash scripts/snapshot_go2w_perception_sidecar.sh
bash scripts/go2w_deepyolo_sidecar.sh stop
```

The sidecar runs the detector and bridge at a lower OS scheduling priority,
records separate PID files and logs under `artifacts/deepyolo_service`, and
refuses to signal a PID unless its command line still matches the expected
process. New detector JSONL files are stored under the service artifact
directory and old streams are pruned to a small retained set. It remains
optional: UI, SLAM startup, localization, and LiDAR-only navigation do not wait
for it.

The generated headless detector and sidecar `resident` profile are paced for
long-term residency by default:

```text
GO2W_DEEPYOLO_PROFILE=resident
GO2W_DEEPYOLO_INPUT_FPS=15
GO2W_DEEPYOLO_IR_MODE=0
GO2W_DEEPYOLO_RENDER_OVERLAY=0
GO2W_DEEPYOLO_CAPTURE_EVERY_N=5
GO2W_DEEPYOLO_INFERENCE_INTERVAL_MS=333
GO2W_DEEPYOLO_HEARTBEAT_MS=1000
GO2W_DEEPYOLO_MAX_JSONL_BYTES=16777216
GO2W_DEEPYOLO_MAX_JSONL_FILES=4
```

This keeps the compatible camera profile at 15 FPS, disables IR streams for the
normal headless semantic path, skips RGB overlay rendering unless an explicit
diagnostic enables it, performs alignment and resize work for every fifth
capture, and caps semantic inference at about 3 Hz. The inference loop
also rejects a repeated latest-frame id so tracking persistence and heartbeat
packets advance only from newly prepared camera frames.
All values can be overridden before `sidecar.sh start`. Lowering camera FPS
alone is not sufficient because the original prototype inference loop may
process the same shared latest frame repeatedly. Profiles below 15 FPS must be
validated on the actual D435I stream combination before use. LiDAR remains the
high-rate safety source; DeepYOLO supplies lower-rate semantic context.

The headless producer writes scene changes immediately and emits a compact
heartbeat at 1 Hz. This keeps source freshness observable during a stable scene
without raising UI refresh rate. Active JSONL streams rotate at 16 MB through
four reused slots; sidecar cleanup trims older cross-session streams on
lifecycle boundaries.

2026-06-01 prone-safe residency benchmark on the current D435I:

| Configuration | Detector CPU | Bridge CPU | Detector memory | Result |
| --- | ---: | ---: | ---: | --- |
| Original headless full-rate prototype | about 93.7% | about 1.1% | about 16.4% | Works, but too expensive for default residency. |
| 15 FPS input, 5 Hz inference cap | about 42.8% | about 0.6% | about 16.4% | CPU improved; capture preparation still expensive. |
| 15 FPS input, prepare every third capture, 5 Hz inference cap | about 28.1% after warm-up | about 0.6% | about 16.4% | Preserved as the `balanced` profile. |
| 6 FPS input | n/a | n/a | n/a | Rejected by the current RGBD+IR D435I stream profile. |

The sidecar exposes explicit profiles so the long-term resident mode can remain
lightweight while a demo can temporarily opt into faster semantic refresh:

```text
resident   -> prepare every fifth capture, about 3 Hz inference, nice 8
balanced   -> prepare every third capture, about 5 Hz inference, nice 5
diagnostic -> prepare every capture, unpaced inference, nice 0
```

The old 5 Hz benchmark remains the `balanced` profile. Measure the current
`resident` profile with `scripts/snapshot_go2w_runtime_resources.sh` after
deploying it on the robot.

Frame-rate controls reduce CPU work but do not release the loaded TensorRT
engine memory. If memory becomes the next bottleneck, benchmark a smaller
TensorRT engine separately rather than weakening SLAM or LiDAR processing.

Set `GO2W_DEEPYOLO_IR_MODE=2` only for an explicit IR diagnostic session. IR
frames are not required for the resident RGBD semantic summary.
Set `GO2W_DEEPYOLO_RENDER_OVERLAY=1` only for a short overlay diagnostic. The
headless semantic JSONL path does not require boxes or labels to be drawn onto
the RGB frame.

The bridge reads the latest `semantic_stream_*.jsonl` packet and writes only a
bounded summary:

```json
{
  "source": "deepyolo_realsense",
  "scene_state": "alert",
  "dominant_class": "person",
  "object_count": 1,
  "high_risk_count": 1,
  "recommended_action": "slow_and_watch",
  "objects": [
    {
      "class_name": "person",
      "risk_level": "high",
      "region": "right",
      "distance_m": 1.89
    }
  ]
}
```

The v1 summary keeps event time and source-file freshness separate:

- `packet_timestamp_ms` and `packet_age_ms` describe the last semantic event;
- `source_file_mtime_ms` and `source_file_age_ms` describe detector output
  health;
- `source_status` distinguishes `fresh`, `event_only_idle`, `stale`,
  `clock_skew`, `depth_insufficient`, and `unavailable`;
- `recommended_action` remains diagnostic context, while `effective_action`
  becomes `ignored` whenever freshness, clock, or depth checks fail.

The JSONL producer is event-oriented rather than heartbeat-oriented. A short
quiet scene therefore becomes `event_only_idle`, not an immediate crash. It is
still excluded from safety actions until a fresh event arrives.

The operator Web UI reads this file through
`--semantic-summary-path` / `GO2W_SEMANTIC_SUMMARY_PATH` and marks it stale with
`--semantic-stale-ms` / `GO2W_SEMANTIC_STALE_MS`.

Current routing policy:

- UI and LLM may see the semantic summary as scene context. The C++ operator
  panel includes `artifacts/stereo_depth_summary.json` and
  `artifacts/vision_semantic_summary.json` in the LLM HTTP payload when those
  files exist.
- `QueueExecutor` and `SafetyGate` do not block on DeepYOLO.
- Future C++ fusion may use high-confidence, fresh semantic objects only to
  increase caution, such as slow/pause when a high-risk center object has valid
  depth. It must never relax LiDAR/SLAM safety.
- If DeepYOLO is absent, stale, slow, or crashes, the main SLAM/LiDAR flow
  continues in LiDAR-only mode.

Before wiring semantics into C++ safety fusion, compare the read-only
`snapshot_go2w_perception_sidecar.sh` output with the sidecar stopped and
running. Record detector FPS plus CPU/GPU load, and keep the navigation-loop
acceptance criteria unchanged: normal runs should keep `poll_overruns == 0` and
`max_loop_elapsed_s < slam_poll_interval_s`.

## Long-Running Subagent Ownership

The project can keep a dedicated StereoDepth/DeepYOLO subagent for this
subsystem. Its ownership boundary should remain narrow:

- maintain the RealSense depth summary and DeepYOLO semantic bridge contracts;
- audit camera latency, stale-data behavior, confidence thresholds, and failure
  modes;
- propose C++ fusion changes only after the summary contract is stable;
- avoid touching SLAM startup, navigation, topology writes, vendor librealsense
  trees, or Unitree `/unitree` configs unless explicitly assigned.

The subagent's normal output should be a short audit report plus a patch plan.
Code changes should use disjoint files from the main operator path to avoid
blocking the real-time runtime work.

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
