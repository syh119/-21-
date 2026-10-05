# jie_ware_humble

`jie_ware-main` core nodes ported to ROS 2 Humble.

Included nodes:

- `lidar_filter_node`: remove isolated outlier points from `LaserScan`
- `costmap_cleaner`: clear Nav2 local/global costmaps after a new initial pose
- `lidar_loc`: map-based laser relocalization node that publishes `map -> odom`

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select jie_ware_humble
source install/setup.bash
```

## Usage

Filter scan:

```bash
ros2 launch jie_ware_humble lidar_filter.launch.py
```

Clear Nav2 costmaps after setting initial pose:

```bash
ros2 launch jie_ware_humble costmap_cleaner.launch.py
```

Run laser relocalization:

```bash
ros2 launch jie_ware_humble lidar_loc.launch.py
```

## Notes

- `lidar_loc` is intended to replace `amcl`. Do not run both at the same time.
- `costmap_cleaner` targets Nav2 services:
  - `/local_costmap/clear_entirely_local_costmap`
  - `/global_costmap/clear_entirely_global_costmap`
- If you enable `lidar_filter_node`, update Nav2 costmap scan topic from `/scan` to `/scan_filtered`.
