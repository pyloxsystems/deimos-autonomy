# DEIMOS Autonomy — Competition Runbook

For the operator deploying this stack at URC 2026 Finals (May 27-30, MDRS Hanksville UT).

This runbook assumes the original autonomy author is **not present** at competition and the team operator runs the stack solo.

---

## Pre-competition checklist (D-1)

- [ ] Verify Jetson is fully powered, fan healthy. `tegrastats` should show GPU + DLA available.
- [ ] Confirm CAN bus up: `ip -d link show can0` shows state UP, bitrate 500000.
- [ ] Confirm Ouster reachable: `ping os1-992005000098.local`.
- [ ] Confirm ZED2 connected: `ZED_Explorer` opens a stream.
- [ ] Confirm GPS lock: `ros2 topic echo /fix --once` shows lat/lon, not 0/0.
- [ ] Confirm all 4 IMUs publishing: `ros2 topic hz /ouster/imu /zed/zed_node/imu/data` etc.
- [ ] DEM tiles loaded: `ls ~/deimos_data/mdrs_dem/*.tif` shows >= 4 tiles covering competition zone.
- [ ] YOLO engine built: `~/deimos_data/yolov8n_urc_int8.engine` (Nano) or `_fp16.engine` (NX).
- [ ] Waypoints file present: `~/deimos_data/urc_waypoints.yaml` with current mission posts.
- [ ] Dry-run launch: `ros2 launch deimos_autonomy autonomy.launch.py compute_profile:=nano_8gb`. All nodes start without errors. Wait 30 s, check `ros2 node list` shows all expected nodes.
- [ ] EKF healthy: `ros2 topic echo /diagnostics --once` no STALE warnings on EKF.
- [ ] RViz2 layout saved: configs in `~/.rviz2/deimos.rviz`.

---

## Competition-day launch sequence

```bash
# 1. Source ROS workspace
source ~/your_ros2_ws/install/setup.bash

# 2. Start hardware (existing flow from robot-repo-ros2)
ros2 launch launch_files/Full_Launch_File.py &

# 3. Start LiDAR (existing)
ros2 launch launch_files/lidar_launch.py &

# 4. Wait 10 s for sensors to settle, then start autonomy
ros2 launch deimos_autonomy autonomy.launch.py compute_profile:=nano_8gb &

# 5. Open RViz monitoring station
rviz2 -d ~/.rviz2/deimos.rviz
```

In RViz watch for:
- `/odometry/filtered` arrow following rover smoothly (no jumps > 0.5 m)
- `/plan` red line from rover to next goal
- `/yolo/annotated` image showing detections (if `publish_debug:=true`)
- `/mission/state` topic shows progression IDLE -> NAVIGATING -> GOAL_REACHED ...

---

## Mission start

Operator publishes the GO signal once the autonomous mission window opens:

```bash
ros2 topic pub --once /mission/cancel std_msgs/Bool "data: false"
# (cancel=false means proceed; goal_sequencer will dispatch first waypoint)
```

To abort mid-mission:

```bash
ros2 topic pub --once /mission/cancel std_msgs/Bool "data: true"
```

---

## Failure modes and responses

### A. EKF drift / odometry jumping

**Symptom:** `/odometry/filtered` arrow jumps wildly in RViz, rover circles in confusion.

**Cause:** One of the 4 IMUs is publishing garbage (stale or NaN), poisoning the EKF.

**Action:**
1. `ros2 topic echo /ouster/imu --once` — verify Ouster IMU sane (orientation w near 1, ang vel < 0.1 idle).
2. Check ZED IMU: `ros2 topic echo /zed/zed_node/imu/data --once`.
3. **If one IMU is bad: edit `config/ekf_local.yaml`, comment out the bad imu block, restart.** Less redundancy but EKF survives.

### B. SLAM lost (Nano profile)

**Symptom:** `/map` stops updating, slam_toolbox warns "scan match failure."

**Cause:** Featureless terrain (flat sand) or LiDAR occluded.

**Action:**
1. Operator commands rover to spin in place via teleop (briefly). Re-acquires features.
2. If persistent, fall back to GPS-only:
   ```bash
   ros2 lifecycle set /slam_toolbox shutdown
   ```
   Nav2 will continue on `/odometry/filtered_map` (GPS-corrected), losing local map but retaining global pose.

### C. Nav2 unable to plan

**Symptom:** `/plan` is empty; `bt_navigator` logs "ComputePathToPose failed."

**Cause:** Goal outside DEM extent, or costmap is fully blocked.

**Action:**
1. Clear costmap: `ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap`.
2. If goal outside DEM: edit `urc_waypoints.yaml` to move waypoint within DEM bounds; restart `goal_sequencer`.

### D. ArUco final-approach loops

**Symptom:** `aruco_action_server` rotates indefinitely, never converges.

**Cause:** Marker ID mismatch, or marker out of camera FoV.

**Action:**
1. Confirm marker_id in `urc_waypoints.yaml` matches the actual URC-provided ID.
2. Increase approach tolerance: edit launch arg `approach_distance:=1.5` (was 1.0).

### E. YOLO engine fails to load

**Symptom:** Logs "Engine file not found" or "TensorRT version mismatch."

**Cause:** Engine compiled on different TRT version than runtime.

**Action:**
1. Rebuild engine on the actual rover Jetson: `bash scripts/build_yolo_trt.sh`.
2. Mission still proceeds — YOLO is non-critical for nav, only for object verification scoring.

### F. Localization disagreement (kill switch)

**Symptom:** EKF outputs jump; FAST-LIO and GPS disagree by > 5 m.

**Cause:** SLAM has lost localization but is still publishing odometry.

**Action:**
1. Goal sequencer should auto-abort with "Localization confidence drop" log.
2. If not auto-abort: operator publishes cancel; falls back to teleop.

---

## Recovery / fallback hierarchy

If autonomy aborts during URC mission, the operator has these options in priority order:

1. **Restart autonomy stack** (10 s):
   ```bash
   pkill -f autonomy.launch.py
   ros2 launch deimos_autonomy autonomy.launch.py compute_profile:=nano_8gb
   ```
2. **Fall back to GPS-waypoint only** (skips SLAM):
   ```bash
   ros2 launch deimos_autonomy autonomy.launch.py \
       compute_profile:=nano_8gb \
       enable_slam:=false
   ```
3. **Manual teleop** — original `joy_mux_controller` flow. Loses 20 % autonomy mission score per URC rules but preserves rover safety and other mission scoring.

---

## Post-mission

```bash
# Save mission rosbag for analysis
ros2 bag record -o urc_mission_$(date +%Y%m%d_%H%M%S) \
    /odometry/filtered /odometry/filtered_map /plan \
    /yolo/detections /aruco_detections /mission/state /tf /tf_static

# Save SLAM map if Nano profile
ros2 run nav2_map_server map_saver_cli -f mdrs_run_$(date +%H%M%S)
```

Email the bag to the autonomy author for post-comp analysis if anomalies seen.

---

## Contact

Emilio Girard / Pylox Forge — pyloxsystems@gmail.com — available remotely throughout the comp window for debug. (Reachable when team is back at hotel with internet; no on-site internet per URC rules.)
