# DEIMOS Autonomy Stack

Drop-in autonomy layer for Space Concordia's DEIMOS rover, targeting URC 2026 Finals.

Implements the architecture described in [docs/AUTONOMY_PROPOSAL.md](docs/AUTONOMY_PROPOSAL.md):

- **SLAM:** FAST-LIO2 (Orin NX path) or slam_toolbox 2D + ZED SDK VIO (Orin Nano path)
- **Sensor fusion:** robot_localization dual-EKF (local + global), fusing 4 onboard IMUs + GPS + SLAM odom
- **Global planning:** USGS DEM A* over MDRS terrain (WVU-style pre-loaded elevation map)
- **Local planning:** Nav2 Regulated Pure Pursuit
- **Final approach:** ArUco visual servo action server
- **Object detection:** YOLOv8n TensorRT (FP16 on DLA / INT8 on GPU)
- **Mission orchestration:** Goal sequencer state machine

Modeled on the publicly documented stacks of WVU Team Mountaineers (2025 URC top autonomy scorer) and AGH Space Systems (2024 URC champion).

---

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- TensorRT 8.5+ (Jetson)
- Python 3.10+

### Install dependencies

```bash
sudo apt update
sudo apt install \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-octomap-server \
    ros-humble-vision-msgs \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-topic-tools

# Python deps for DEM planner
pip install rasterio pyproj numpy opencv-python

# Optional: ZED SDK (Stereolabs official installer)
# Optional: TensorRT + pycuda for YOLO
pip install pycuda
```

### Build into existing workspace

Drop `src/deimos_autonomy` next to the existing `src/` packages from `robot-repo-ros2`.

```bash
cd ~/your_ros2_ws/src
ln -s /path/to/deimos-autonomy/src/deimos_autonomy .
cd ..
colcon build --symlink-install --packages-select deimos_autonomy
source install/setup.bash
```

### Run

```bash
# Orin Nano profile (slam_toolbox 2D + ZED VIO)
ros2 launch deimos_autonomy autonomy.launch.py compute_profile:=nano_8gb

# Orin NX profile (FAST-LIO2 LiDAR-inertial)
ros2 launch deimos_autonomy autonomy.launch.py compute_profile:=nx_16gb

# With Gazebo simulation
ros2 launch deimos_autonomy autonomy.launch.py compute_profile:=nano_8gb use_sim_time:=true
```

### Pre-flight diagnostic

Before integrating, run the read-only diagnostic on your rover:

```bash
python3 scripts/deimos_preflight.py
```

It validates ROS2 topics, TF tree, CAN bus, Ouster reachability, ZED2 USB,
power mode, RAM/disk headroom, and JetPack version. Pass/warn/fail report
with actionable fixes. Sample report at `docs/SAMPLE_PREFLIGHT_REPORT.txt`.

---

## Topic graph

### Consumed (existing topics, see `robot-repo-ros2`)

| Topic | Type | Source |
|---|---|---|
| `/ouster/points` | sensor_msgs/PointCloud2 | `ouster_ros` (lidar_launch.py) |
| `/ouster/imu` | sensor_msgs/Imu | `ouster_ros` |
| `/zed/zed_node/rgb/image_rect_color` | sensor_msgs/Image | `zed_wrapper` |
| `/zed/zed_node/imu/data` | sensor_msgs/Imu | `zed_wrapper` |
| `/zed/zed_node/odom` | nav_msgs/Odometry | ZED SDK Positional Tracking (Nano profile) |
| `/fix` | sensor_msgs/NavSatFix | `ublox_gps_node` |
| `/aruco_detections` | aruco_opencv_msgs/ArucoDetection | `ros_aruco_opencv` |

### Published (this package)

| Topic | Type | Producer |
|---|---|---|
| `/odometry/dlio` | nav_msgs/Odometry | FAST-LIO2 or ZED-VIO bridge |
| `/odometry/filtered` | nav_msgs/Odometry | EKF local |
| `/odometry/filtered_map` | nav_msgs/Odometry | EKF global |
| `/odometry/gps` | nav_msgs/Odometry | navsat_transform_node |
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox (Nano only) |
| `/scan` | sensor_msgs/LaserScan | pointcloud_to_laserscan (Nano only) |
| `/plan` | nav_msgs/Path | dem_global_planner |
| `/traversability` | nav_msgs/OccupancyGrid | dem_global_planner (debug) |
| `/yolo/detections` | vision_msgs/Detection2DArray | yolo_trt_node |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 controller / aruco_action_server |
| `/mission/state` | std_msgs/String | goal_sequencer |

---

## File map

```
deimos_autonomy/
├── docs/
│   ├── AUTONOMY_PROPOSAL.md          ← Architecture overview, design rationale, references
│   └── COMPETITION_RUNBOOK.md        ← Deployment, troubleshooting, fallback procedures
├── src/deimos_autonomy/
│   ├── package.xml
│   ├── setup.py
│   ├── launch/
│   │   ├── autonomy.launch.py        ← Master launch with compute_profile switching
│   │   ├── fast_lio.launch.py        ← FAST-LIO2 (NX path)
│   │   ├── slam_toolbox_2d.launch.py ← slam_toolbox + ZED VIO (Nano path)
│   │   ├── ekf.launch.py             ← Dual-EKF + navsat_transform
│   │   └── nav2.launch.py            ← Nav2 lifecycle nodes
│   ├── config/
│   │   ├── fast_lio_ouster.yaml      ← Ouster OS1 tuning
│   │   ├── slam_toolbox_2d.yaml
│   │   ├── ekf_local.yaml            ← 4-IMU fusion
│   │   ├── ekf_global.yaml           ← + GPS
│   │   ├── navsat_transform.yaml
│   │   ├── nav2_params.yaml          ← Nav2 + costmaps
│   │   └── nav2_bt_navigate.xml      ← Behavior tree
│   └── deimos_autonomy/
│       ├── aruco_action_server.py    ← Final-approach visual servo
│       ├── yolo_trt_node.py          ← TRT YOLOv8n inference
│       ├── dem_global_planner.py     ← USGS DEM A*
│       └── goal_sequencer.py         ← Mission state machine
└── scripts/
    ├── train_yolo_urc.py             ← Fine-tune YOLOv8n on URC objects (DGX Spark)
    ├── build_yolo_trt.sh             ← ONNX -> TRT engine (Jetson)
    ├── download_mdrs_dem.sh          ← USGS National Map fetch
    └── record_demo_bag.sh            ← Record sample data for Gazebo replay
```

---

## License

PolyForm Noncommercial 1.0.0. See LICENSE. Free for research, education, and noncommercial use. Commercial use requires a separate license — contact pyloxsystems@gmail.com.

---

## Author

Emilio Girard / Pylox Forge — pyloxsystems@gmail.com
