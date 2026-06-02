# GO2W XT16 Geometry Calibration Notes

## Static probe result

The robot remained prone and no motion command was sent during this probe.

On 2026-06-02, the XT16 driver was alive and receiving Ethernet traffic. The
raw point-cloud topic `/unitree/slam_lidar/points` produced
`sensor_msgs/msg/PointCloud2` frames with:

- `frame_id=rslidar`
- approximately `51k` points per frame
- fields `x`, `y`, `z`, `intensity`, `ring`, and `time`
- `point_step=22`

The vendor-derived topics such as `/collision_clouds`, `/pre_collision_clouds`,
`/safe_clouds`, and `/utlidar/cloud_base` did not produce a static sample in
this state.

## Why raw XT16 is not yet an execution gate

The raw cloud contains many near-body returns. A naive minimum-distance rule
would classify robot self-reflections as obstacles. The side distributions
also need a physical obstacle placement check before assuming the sign and
origin of the `rslidar` axes.

Do not authorize movement from an uncalibrated XT16 summary. The current D435
ROI summary remains an additional fail-closed safety floor, not a replacement
for a calibrated 360-degree source.

## Required prone calibration

1. Keep execution disabled and the robot prone.
2. Capture a baseline cloud with no movable obstacle near the body.
3. Place a box at measured distances in front, left, right, and rear in turn.
4. Confirm the `rslidar` axis directions and LiDAR-to-body transform.
5. Define a body footprint exclusion box and height band that remove self
   returns without hiding the measured box.
6. Validate clearance estimates against the measured distances.
7. Emit a compact `lidar_pointcloud` JSON summary at a bounded rate.
8. Fuse XT16 and D435 conservatively: a source may reduce clearance, never
   increase another source's clearance.
9. Re-run prone blocking checks before permitting a `0.2m` open-area trial.

## Startup probe correction

`scripts/start_go2w_slam_stack.sh` now treats a topic as ready after the first
received output line. The previous probe waited for `ros2 topic echo` to exit,
which caused a false warning because that command is intentionally continuous.

