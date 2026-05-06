# DEIMOS Rover — Autonomy Stack Proposal

**Target:** URC 2026 Finals (May 27-30, 2026, MDRS Utah)
**Author:** Emilio Girard (Pylox Forge / pyloxsystems@gmail.com)
**Date:** 2026-05-06

---

## Problem statement

Space Concordia's `robot-repo-ros2` has Nav2 listed as a dependency but **no localization, SLAM, or autonomous navigation node is launched anywhere in the repo**. Issue #38 (SLAM Epic) has been open and unassigned since August 2025. Issue #58 admits Nav2 + SLAM parameters are broken. The autonomous navigation mission is worth 100 points (20% of URC scoring); the rover currently scores zero on it.

Meanwhile, the rover is equipped with championship-class sensors:
- Ouster OS1 LiDAR (3D, with built-in 9-DOF IMU)
- Stereolabs ZED2 stereo camera (with 6-DOF IMU)
- u-blox GPS
- CAN-bus motor control with absolute encoders

The hardware is in place. The integration is the gap.

---

## Architecture overview

This proposal delivers a deployable autonomy layer modeled on the **WVU Team Mountaineers stack** (URC 2025 second-place finisher and top autonomy scorer) combined with **AGH Space Systems** (URC 2024 champion) sensor-fusion patterns. Both stacks are publicly documented; this proposal applies them to DEIMOS's specific topic graph and Jetson compute tier.

```
                +-------------------+
                |   Ouster OS1      |  /ouster/points     +-------------+
                |   (3D LiDAR+IMU)  |  /ouster/imu  ---->|             |
                +-------------------+                     |  FAST-LIO2  |--> /odometry/dlio
                                                          |             |
                +-------------------+                     +-------------+
                |   ZED2 Stereo     |  /zed/.../imu/data        |
                |   (RGB + IMU)     |                           v
                +-------------------+                    +---------------+
                          |                              |  EKF (local)  |--> /odometry/filtered
                          | /zed/.../rgb/image_rect      |               |
                          v                              +---------------+
                +-------------------+                           |
                | YOLOv8n + ArUco   |                           |
                | (TRT FP16/INT8)   |                           v
                +-------------------+                    +---------------+
                          |                              |  EKF (global) |--> /odometry/filtered_map
                          v                              |  + GPS        |
                +-------------------+                    +---------------+
                | ArUco Action Srv  |       /fix                |
                | (final approach)  |        |                  |
                +-------------------+        |                  v
                          |                  |          +---------------+
                          v                  +--------->|     Nav2      |
                +-------------------+                   |  + DEM global |
                | Goal sequencer    |------------------>|     planner   |
                | (URC waypoints)   |                   +---------------+
                +-------------------+                           |
                                                                v
                                                          /cmd_vel
                                                                |
                                                                v
                                                    wheels_controller_node
                                                          (existing)
```

---

## Stack — locked components

### 1. FAST-LIO2 (LiDAR-Inertial Odometry)

**Why:** WVU Team Mountaineers' deployed stack at URC 2025. Tight-coupled iterated Kalman filter. Ouster OS1 is fully supported. Real-time on Jetson Orin NX (~20-50 Hz).

**Inputs:** `/ouster/points`, `/ouster/imu`
**Outputs:** `/odometry/dlio`, `/dlio/odom_node/path`, `/dlio/odom_node/keyframes`

**Reference:** github.com/hku-mars/FAST_LIO

### 2. robot_localization Dual-EKF

**Why:** AGH Space Systems' confirmed pattern. Mature, ROS2-native, YAML-only configuration.

- **Local EKF** (`ekf_local.yaml`): fuses FAST-LIO2 odom + ZED2 IMU + Ouster IMU → `/odometry/filtered` (continuous, smooth).
- **Global EKF** (`ekf_global.yaml`): adds u-blox GPS via `navsat_transform_node` → `/odometry/filtered_map` (globally-anchored).

**Note:** The team has no wheel odometry publisher (verified — `wheels_controller` subscribes to `/cmd_vel` but does not publish `/odom`). This proposal does not require wheel odom; FAST-LIO2 + dual-IMU + GPS is sufficient.

### 3. Nav2 + USGS DEM Global Planner

**Why:** Nav2 is already an apt dependency in the repo's README. WVU's secret weapon at MDRS is pre-loading USGS Digital Elevation Model tiles for the competition zone and running A* over the DEM as the global planner. Local planner uses costmap from FAST-LIO2 occupancy.

**Custom node:** `dem_global_planner` (Python) — loads MDRS GeoTIFF tiles, projects waypoints into DEM frame, runs A* with traversability cost (slope + obstacle), publishes `/plan`.

### 4. ArUco Final-Approach Action Server

**Why:** URC autonomy mission requires <2m precision at GPS-flagged posts and visual identification of fiducials. The team's `ros_aruco_opencv` already detects markers; this wraps detection in a Nav2-compatible action server that drives the rover the final 3-5m via visual servoing once Nav2 has gotten within GPS tolerance.

**Inputs:** `/zed/zed_node/rgb/image_rect_color` (existing topic), ArUco detections
**Outputs:** Action `FinalApproach.action` callable from the goal sequencer.

### 5. YOLOv8n on TensorRT (Object Detection)

**Why:** URC autonomy mission requires identifying specific objects (orange rubber mallet, rock pick, water bottle). YOLOv8n fine-tuned on synthetic + Mars-desert backgrounds, exported to TensorRT.

- **NX 16GB profile:** FP16 engine, runs on DLA cores (frees GPU for FAST-LIO2). Realistic ~100+ FPS.
- **Nano 8GB profile:** INT8 engine, runs on GPU. Realistic ~30-60 FPS.

Hand off as `.engine` file + ROS2 inference node consuming `/zed/zed_node/rgb/image_rect_color`.

### 6. Goal Sequencer

State machine that consumes URC mission waypoints (pre-loaded), drives the autonomy hierarchy:
1. Receive next waypoint
2. Use DEM global planner + Nav2 to navigate to GPS coordinates
3. Within 3m of waypoint, hand off to ArUco action server for fiducial servoing
4. Within 1m of fiducial, run YOLO verification on target object
5. Confirm or abort, advance to next waypoint

---

## Compute profiles

The same codebase ships with two launch profiles selected via `compute_profile` argument.

### `compute_profile:=nx_16gb` (PRIMARY)

Full FAST-LIO2 LiDAR-inertial stack. YOLO on DLA. Recommended.

```bash
ros2 launch deimos_autonomy autonomy.launch.py compute_profile:=nx_16gb
```

### `compute_profile:=nano_8gb` (FALLBACK)

ZED SDK Positional Tracking replaces FAST-LIO2 (lighter on CPU+RAM).
slam_toolbox 2D consumes Ouster scan via pointcloud_to_laserscan.
YOLO INT8 on GPU.

```bash
ros2 launch deimos_autonomy autonomy.launch.py compute_profile:=nano_8gb
```

---

## Topic remapping table

This package consumes/publishes against the team's existing topic graph. Verified from `robot-repo-ros2/launch_files/Full_Launch_File.py` and `lidar_launch.py`.

| Consumed (their existing) | Provided by | Notes |
|---|---|---|
| `/ouster/points` | `lidar_launch.py` (ouster_ros) | 3D point cloud |
| `/ouster/imu` | ouster_ros sensor.launch.xml | OS1 built-in 9-DOF IMU |
| `/zed/zed_node/rgb/image_rect_color` | zed_wrapper | Confirmed in Full_Launch_File.py:55 |
| `/zed/zed_node/imu/data` | zed_wrapper | ZED2 6-DOF IMU |
| `/zed/zed_node/depth/depth_registered` | zed_wrapper | Optional, for OctoMap |
| `/fix` | ublox_gps_node (default) | NavSatFix |

| Published (this package) | Consumer |
|---|---|
| `/odometry/dlio` | EKF local |
| `/odometry/filtered` | Nav2 |
| `/odometry/filtered_map` | DEM global planner |
| `/map` | Nav2 costmap (slam_toolbox profile only) |
| `/plan` | Nav2 controller |
| `/cmd_vel` | wheels_controller_node (existing) |

---

## Failover hierarchy

The stack is designed to degrade gracefully under sensor failure:

1. **Primary:** FAST-LIO2 LiDAR-inertial odometry (sub-10cm local accuracy)
2. **Secondary:** ZED SDK Positional Tracking (visual-inertial)
3. **Tertiary:** GPS-only with IMU dead-reckoning between fixes
4. **Manual abort:** if odometry sources disagree by > threshold for > 3s, autonomy aborts and returns control to operator with clear error

This is critical because the operator may not be present at competition site to debug. The stack must fail safely without intervention.

---

## Build + deployment

```bash
# On Jetson, in their workspace
cd ~/robot-repo-ros2
git remote add deimos-autonomy <fork-url>
git fetch deimos-autonomy
git checkout -b autonomy-integration deimos-autonomy/main

# Install autonomy deps
sudo apt install ros-humble-robot-localization ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan ros-humble-octomap-server \
    ros-humble-zed-ros2-wrapper

# FAST-LIO2 (manual build)
cd ~/ros2_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd .. && colcon build --packages-select fast_lio --symlink-install

# Build everything
colcon build --symlink-install
source install/setup.bash

# Launch
ros2 launch deimos_autonomy autonomy.launch.py compute_profile:=nx_16gb
```

---

## What this proposal does NOT do

- Does not modify any existing teleoperation flow. The autonomy launch is additive; running the existing `Full_Launch_File.py` continues to work unchanged.
- Does not require hardware changes.
- Does not require internet at the competition site (URC rules confirmed prohibit internet — see urc.marssociety.org/home/requirements-guidelines/qa).
- Does not bet the competition on autonomy. The kill switch returns control to operator if the stack fails.

---

## Open questions

These need confirmation before final tuning:

1. **Exact Jetson model.** Orin Nano 8GB or NX 16GB? Determines which profile is primary.
2. **Ouster OS1 beam count.** OS1-32, 64, or 128? Affects FAST-LIO2 voxel filter tuning.
3. **GPS topic name.** Default `/fix` or remapped? `ublox_gps_node` defaults assumed.
4. **ZED2 calibration state.** Has the ZED been calibrated against base_link? Required for VIO accuracy.
5. **TF tree current state.** What frames exist? `base_link`, `odom`, `map` standard? Sensor mount frames documented?

---

## References

- [WVU Team Mountaineers SAR PDF](https://urc.orgs.wvu.edu/files/d/063786fe-e7ac-4d1f-b98e-41602ac90b74/team-mountaineers-system-acceptance-review.pdf) — FAST-LIO2 + USGS DEM stack
- [AGH Space Systems kalman_robot](https://github.com/agh-space-systems-rover/kalman_robot) — RTAB-Map + robot_localization + Nav2
- [FAST-LIO2 paper](https://arxiv.org/abs/2107.06829)
- [robot_localization documentation](http://docs.ros.org/en/humble/p/robot_localization/)
- [Nav2 documentation](https://navigation.ros.org/)
- [URC 2026 Rules + Q&A](https://urc.marssociety.org/home/requirements-guidelines/qa)
