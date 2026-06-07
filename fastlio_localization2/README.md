# fast_lio_localization

Pure ROS 2 relocalization package for an external FAST-LIO2 frontend.

This package no longer builds, launches, or depends on the bundled FAST-LIO
mapping frontend. It consumes the aligned scan and odometry already published by
an external `fast_lio` process, runs ICP against a horizontal PCD map, and
publishes the `map -> odom` correction.

Expected TF tree:

```text
map
  `-- odom
        `-- base_link
```

- `map -> odom`: published by this package.
- `odom -> base_link`: published by the external FAST-LIO2 frontend.
- This package does not publish `odom -> base_link`.
- This package does not use `camera_init` or `body`.

## Inputs

Start an external FAST-LIO2 frontend first. It must provide:

```text
/cloud_registered_aligned  sensor_msgs/PointCloud2  header.frame_id=odom
/nav_odom                  nav_msgs/Odometry        header.frame_id=odom, child_frame_id=base_link
odom -> base_link          TF
```

The map must be in the same horizontal coordinate system as
`/cloud_registered_aligned`, for example:

- a FAST-LIO2 `*_aligned.pcd` map
- a PGO optimized horizontal `map.pcd`

## Outputs

```text
/map_to_odom   nav_msgs/Odometry  header.frame_id=map, child_frame_id=odom
/localization  nav_msgs/Odometry  header.frame_id=map, child_frame_id=base_link
map -> odom    TF
```

## Dependencies

ROS 2 dependencies are declared in `package.xml`.

Python dependencies used by the ICP relocalizer:

```bash
python3 -m pip install --user open3d
```

`scipy` is used for quaternion and matrix conversion. On Ubuntu/ROS systems it
is usually available from apt:

```bash
sudo apt install python3-scipy
```

This package does not require `tf_transformations` or `ros2_numpy`.

## Build

From the workspace root:

```bash
cd ~/fastlio_ws
colcon build --packages-select fast_lio_localization
source install/setup.bash
```

## Run

Terminal 1, start the external FAST-LIO2 frontend:

```bash
ros2 launch fast_lio mapping.launch.py use_rviz:=false
```

Verify the external frontend:

```bash
ros2 topic echo /nav_odom --once
ros2 topic echo /cloud_registered_aligned --once
```

Expected:

```text
/nav_odom.header.frame_id = odom
/nav_odom.child_frame_id = base_link
/cloud_registered_aligned.header.frame_id = odom
```

Terminal 2, start relocalization:

```bash
ros2 launch fast_lio_localization localization.launch.py \
  pcd_map_path:=/home/zgm777/fastlio_ws/test_aligned.pcd \
  scan_topic:=/cloud_registered_aligned \
  odom_topic:=/nav_odom \
  map_frame:=map \
  odom_frame:=odom \
  base_frame:=base_link
```

In RViz, use `2D Pose Estimate` to publish `/initialpose`. The initial pose is
interpreted as `map -> base_link`. The node computes:

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

## TF Checks

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

`map -> base_link` should be resolved through:

```text
map -> odom -> base_link
```

Do not run another node that also publishes `map -> odom` at the same time.
