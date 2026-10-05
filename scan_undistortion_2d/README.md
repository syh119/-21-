# scan_undistortion_2d

Lightweight ROS2 deskew node for 2D single-line lidar.

Use case:

- input topic type: `sensor_msgs/msg/LaserScan`
- single-line 2D lidar
- robot provides `sensor_msgs/msg/Imu`

Current behavior:

- input: `/scan` + `/imu/data_raw`
- optional odom input: `/odom`
- output: `/scan_deskewed`
- point cloud output: `/scan_deskewed_cloud`
- raw cloud output: `/scan_origin_cloud`
- uses IMU yaw interpolation to correct rotational distortion
- if IMU orientation is unavailable, it can estimate orientation internally from raw IMU
- can additionally use odom translation to compensate in-frame XY motion
- extracts yaw only from IMU, matching the original `2d_lidar_undistortion` design
- estimates `scan_time` and `time_increment` when the lidar driver leaves them as zero

Default parameters:

- `scan_topic=/scan`
- `imu_topic=/imu/data_raw`
- `odom_topic=/odom`
- `output_topic=/scan_deskewed`
- `output_cloud_topic=/scan_deskewed_cloud`
- `raw_cloud_topic=/scan_origin_cloud`
- `oriented_imu_topic=/imu/oriented`
- `default_scan_rate_hz=10.0`
- `lidar_msg_delay_ms=10.0`
- `imu_buffer_sec=3.0`
- `odom_buffer_sec=3.0`
- `laser_offset_x=0.0`
- `laser_offset_y=0.0`
- `scan_direction_clockwise=false`
- `publish_raw_cloud=true`
- `use_odom_translation=true`

Build:

```bash
colcon build --packages-select scan_undistortion_2d
```

Run:

```bash
ros2 launch scan_undistortion_2d scan_undistortion.launch.py
```

For `origincar + lslidar`, you can feed this node directly with raw IMU data
from `/imu660ra/data_raw`. When the incoming IMU does not provide a valid
`orientation` field, this package can estimate orientation internally from the
raw gyro and accel data, then use that result for deskew.

With `use_odom_translation=true`, the node uses:

- IMU yaw for rotation deskew
- odom `x/y` position interpolation for translation deskew

If the IMU topic does not publish a valid `orientation` field, the node can
run an internal Madgwick-style orientation estimator from raw IMU data. This is
enabled by default through `estimate_orientation_from_raw_imu=true`.
The estimated orientation is also published as a new IMU topic through
`oriented_imu_topic` so it can be inspected or reused by other nodes.

If your lidar is offset from the vehicle rotation center, set `laser_offset_x`
and `laser_offset_y` to compensate the extra apparent motion caused by pure
rotation. For the current `origincar` TF, `laser_offset_x` is `0.105`.

If your odom is too noisy, you can fall back to IMU-only mode:

```bash
ros2 launch scan_undistortion_2d scan_undistortion.launch.py use_odom_translation:=false
```

Recommended `origincar + imu660ra` launch:

```bash
ros2 launch scan_undistortion_2d scan_undistortion.launch.py \
  imu_topic:=/imu660ra/data_raw \
  odom_topic:=/odom \
  use_odom_translation:=false \
  laser_offset_x:=0.105
```

If Nav2 should consume the deskewed scan, change `scan_topic` from `/scan` to
`/scan_deskewed` in the Nav2 config.

To visualize the deskewed planar cloud in RViz, add a `PointCloud2` display and
select `/scan_deskewed_cloud`.
