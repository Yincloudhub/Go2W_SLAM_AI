# GO2W multisensor collection

This collector records one ROS 2 sqlite3 bag after verifying that each required
topic produces an actual sample.

Required roles and preferred topics:

- front camera: `/frontvideostream`
- point cloud: `/utlidar/cloud`
- IMU: `/utlidar/imu`
- odometry: `/utlidar/robot_odom`
- attitude: `/utlidar/robot_pose`

The collector automatically tries known fallback odometry and attitude topics.
It also records `/sportmodestate` when that topic is active.

Deploy this directory to the dog NX, for example under:

```text
/home/unitree/NX_radar_fleet/collection
```

Run:

```bash
bash ~/NX_radar_fleet/collection/collect_go2w_multisensor.sh \
  --name jiaofan08 \
  --duration 30
```

When one role has no live data, the collector continues recording all other
online topics. It writes Chinese status feedback to:

```text
~/go2w_dataset/collection_status.json
```

The operator Web UI reads this file and displays the missing data roles. After
recording, the collector checks `metadata.yaml` and reports each topic count.

For a deliberate camera/cloud/IMU-only capture:

```bash
bash ~/NX_radar_fleet/collection/collect_go2w_multisensor.sh \
  --name test_no_state \
  --duration 10 \
  --allow-missing-state
```
