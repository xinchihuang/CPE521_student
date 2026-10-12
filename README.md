# Rawseeds SLAM Student Framework

Minimal ROS 2 Jazzy framework for implementing a 2D SLAM pose estimator on a recorded Rawseeds sample bag.

Students only need to edit:

```text
src/rawseeds_slam/rawseeds_slam/student_algorithm.py
```

Runnable example methods live in:

```text
src/rawseeds_slam/rawseeds_slam/slam/
```

## Build

```bash
cd /home/xinchi/CPE521_ROS2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run the sample data

This opens RViz and replays the included 60 second bag.

```bash
ros2 launch rawseeds_slam slam.launch.py \
  mode:=data bag:="$PWD/bags/sample_60s" rate:=0.5
```

## Run your implementation

```bash
ros2 launch rawseeds_slam slam.launch.py \
  mode:=student bag:="$PWD/bags/sample_60s" rate:=0.5 \
  trajectory:="$PWD/results/student_trajectory.csv"
```

`student_algorithm.py` must provide a `PoseEstimator` class with this method:

```python
def update(self, points_laser, odom_base, base_to_laser, timestamp):
    return pose_map_base  # numpy array [x, y, yaw]
```

Inputs:

- `points_laser`: finite scan endpoints in the laser frame, shape `N x 2`
- `odom_base`: odometry pose of `base_link`, `[x, y, yaw]`
- `base_to_laser`: fixed laser extrinsic, `[x, y, yaw]`
- `timestamp`: scan time in seconds

Return the estimated `base_link` pose in the map frame as `[x, y, yaw]`.

## Run the demo method

```bash
ros2 launch rawseeds_slam slam.launch.py \
  mode:=demo bag:="$PWD/bags/sample_60s" rate:=0.5 \
  trajectory:="$PWD/results/demo_trajectory.csv"
```

The demo uses `rawseeds_slam/slam/scan_matcher.py`. It is a compact scan-matching example, not a tuned benchmark.

## Useful files

| Path | Purpose |
|---|---|
| `bags/sample_60s/` | Included ROS 2 bag for testing |
| `src/rawseeds_slam/launch/slam.launch.py` | Launch file for data, student, and demo modes |
| `src/rawseeds_slam/config/sensors.yaml` | Laser extrinsics |
| `src/rawseeds_slam/rawseeds_slam/student_algorithm.py` | Student implementation entry point |
| `src/rawseeds_slam/rawseeds_slam/slam/` | Example implemented methods |
| `src/rawseeds_slam/rawseeds_slam/mapping.py` | Occupancy-grid helper |
| `tests/` | Small offline tests for the framework |

## Test

```bash
python3 -m pytest -q tests
```
