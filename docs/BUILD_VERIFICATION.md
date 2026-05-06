# Build Verification

The `deimos_autonomy` package was built in a clean `ros:humble` Docker
container on 2026-05-06 to verify no missing CMake patches or build-time
issues block a fresh `colcon build`.

## Result: PASS

```
Starting >>> deimos_autonomy
Finished <<< deimos_autonomy [0.54s]
Summary: 1 package finished [0.67s]
```

All four entry points installed and discoverable:

```
/ws/install/deimos_autonomy/bin/dem_global_planner
/ws/install/deimos_autonomy/bin/aruco_action_server
/ws/install/deimos_autonomy/bin/yolo_trt_node
/ws/install/deimos_autonomy/bin/goal_sequencer
```

`import deimos_autonomy` works after sourcing `install/setup.bash`.

## Reproducing

```bash
docker run --rm \
    -v $(pwd)/src/deimos_autonomy:/mnt/deimos_autonomy:ro \
    --entrypoint bash \
    ros:humble \
    -c '
mkdir -p /ws/src
cp -r /mnt/deimos_autonomy /ws/src/deimos_autonomy
cd /ws
apt-get update -qq && apt-get install -y -qq \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-vision-msgs \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-topic-tools \
    python3-pip
source /opt/ros/humble/setup.bash
colcon build --packages-select deimos_autonomy
'
```

## Runtime dependency status (post-build)

When each entry point starts, it logs the runtime dependencies it needs.
On a clean container these dependencies are intentionally absent and the
package degrades gracefully rather than failing silently. On the actual
DEIMOS rover Jetson these will be present.

| Node | Missing in clean container | Available on rover? | Required for |
|------|----------------------------|---------------------|--------------|
| dem_global_planner | `rasterio` (Python) | `pip install rasterio` | DEM tile loading + traversability A* |
| aruco_action_server | `aruco_opencv_msgs` | yes (team's `ros_aruco_opencv`) | Final-approach visual servo |
| yolo_trt_node | `tensorrt`, `pycuda` | yes (JetPack-bundled) | TRT engine inference |
| goal_sequencer | `urc_waypoints.yaml` | provide via launch arg | Mission orchestration |

## Known runtime-dependent items

These were verified to launch cleanly but assert specific environment
state at runtime. The pre-flight diagnostic
(`scripts/deimos_preflight.py`) catches all of them in 30 seconds against
the actual rover.

- TF tree must contain `base_link`, `odom`, `map`, `ouster_lidar`,
  `zed2_camera_center`. Diagnostic checks frame presence.
- `ublox_gps_node` topic name is assumed to be `/fix` (default).
- `aruco_opencv_msgs.msg.ArucoDetection` field structure assumed; the
  ArUco action server has a graceful import-fallback if this differs.
- ZED SDK Positional Tracking must be enabled in `zed_camera.launch.py`
  on the Nano profile (publishes `/zed/zed_node/odom`). Diagnostic warns
  if the topic is not present.

## What this verification does NOT cover

- Live integration on the actual rover (requires physical hardware)
- EKF covariance tuning against real sensor noise
- FAST-LIO2 build (separate `git clone` + manual build, see README install
  section)
- Validating ZED SDK install on JetPack 6.x (Stereolabs official
  installer)

These are the "last 20%" referenced in the README and the cold email.
The pre-flight diagnostic identifies which of these need attention on
your specific rover before integration.
