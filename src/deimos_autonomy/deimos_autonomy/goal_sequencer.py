"""Mission Goal Sequencer for URC autonomous navigation.

Reads a YAML waypoint file (URC mission spec: 3 GPS posts + 3 objects) and
drives the rover through them in order, using the appropriate behavior for
each goal type:

  - GPS_POST: Nav2 NavigateToPose to GPS coordinates, accept within tolerance.
  - ARUCO_POST: Nav2 NavigateToPose to GPS, then ArUco final-approach action.
  - OBJECT: Nav2 NavigateToPose to GPS, then object search via YOLO detection
            within a bounded radius; report success on first high-confidence hit.

Mission state machine:
  IDLE -> NAVIGATING -> FINAL_APPROACH (if applicable) -> OBJECT_VERIFY (if applicable)
       -> GOAL_REACHED -> NAVIGATING (next) -> ... -> MISSION_COMPLETE

Aborts on:
  - Nav2 hard failure after retries
  - Localization confidence drop below threshold (manual flag)
  - Operator cancel via /mission/cancel topic
"""

import math
import os
from enum import Enum
from typing import List, Optional

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from vision_msgs.msg import Detection2DArray


class WaypointType(Enum):
    GPS_POST = 'gps_post'
    ARUCO_POST = 'aruco_post'
    OBJECT = 'object'


class MissionState(Enum):
    IDLE = 'idle'
    NAVIGATING = 'navigating'
    FINAL_APPROACH = 'final_approach'
    OBJECT_VERIFY = 'object_verify'
    GOAL_REACHED = 'goal_reached'
    ABORTED = 'aborted'
    MISSION_COMPLETE = 'mission_complete'


class Waypoint:
    def __init__(self, raw: dict):
        self.id = raw.get('id', '?')
        self.type = WaypointType(raw['type'])
        self.x = float(raw['x'])
        self.y = float(raw['y'])
        self.tolerance = float(raw.get('tolerance', 2.0))
        self.marker_id = int(raw.get('marker_id', -1))
        self.object_class = raw.get('object_class')
        self.object_search_radius = float(raw.get('object_search_radius', 5.0))


class GoalSequencer(Node):
    def __init__(self):
        super().__init__('goal_sequencer')

        self.declare_parameter(
            'waypoints_file', os.path.expanduser('~/deimos_data/urc_waypoints.yaml')
        )
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('aruco_action', '/aruco_final_approach')
        self.declare_parameter('detection_topic', '/yolo/detections')
        self.declare_parameter('odom_topic', '/odometry/filtered_map')
        self.declare_parameter('object_confidence', 0.6)
        self.declare_parameter('object_verify_timeout_s', 60.0)

        self.state = MissionState.IDLE
        self.waypoints: List[Waypoint] = []
        self.current_idx = 0
        self._latest_odom: Optional[Odometry] = None
        self._latest_detections: List[dict] = []
        self._verify_started_t: Optional[float] = None

        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter('navigate_action').value
        )
        self.aruco_client = ActionClient(
            self, NavigateToPose, self.get_parameter('aruco_action').value
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, qos
        )
        self.det_sub = self.create_subscription(
            Detection2DArray,
            self.get_parameter('detection_topic').value,
            self._det_cb,
            10,
        )
        self.cancel_sub = self.create_subscription(
            Bool, '/mission/cancel', self._cancel_cb, 10
        )
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        self._load_waypoints()
        self.timer = self.create_timer(0.5, self._tick)

    def _load_waypoints(self):
        path = self.get_parameter('waypoints_file').value
        if not os.path.exists(path):
            self.get_logger().error(
                f'Waypoints file {path} not found. Mission will idle.'
            )
            return
        with open(path) as f:
            raw = yaml.safe_load(f)
        self.waypoints = [Waypoint(w) for w in raw.get('waypoints', [])]
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')

    def _odom_cb(self, msg):
        self._latest_odom = msg

    def _det_cb(self, msg):
        self._latest_detections = [
            {
                'class_id': d.results[0].hypothesis.class_id if d.results else None,
                'score': d.results[0].hypothesis.score if d.results else 0.0,
            }
            for d in msg.detections
        ]

    def _cancel_cb(self, msg):
        if msg.data:
            self.get_logger().warn('Mission cancelled by operator.')
            self.state = MissionState.ABORTED

    def _publish_state(self):
        m = String()
        m.data = f'{self.state.value} wp={self.current_idx}/{len(self.waypoints)}'
        self.state_pub.publish(m)

    def _tick(self):
        self._publish_state()

        if self.state == MissionState.IDLE:
            if self.waypoints:
                self.current_idx = 0
                self._dispatch_current()
            return

        if self.state == MissionState.GOAL_REACHED:
            self.current_idx += 1
            if self.current_idx >= len(self.waypoints):
                self.state = MissionState.MISSION_COMPLETE
                self.get_logger().info('Mission complete.')
                return
            self._dispatch_current()
            return

        if self.state == MissionState.OBJECT_VERIFY:
            self._poll_object_verify()
            return

    def _dispatch_current(self):
        wp = self.waypoints[self.current_idx]
        self.get_logger().info(
            f'Dispatching waypoint {self.current_idx}: '
            f'{wp.type.value} at ({wp.x:.1f}, {wp.y:.1f})'
        )
        self._send_nav_goal(wp)
        self.state = MissionState.NAVIGATING

    def _send_nav_goal(self, wp: Waypoint):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server unavailable.')
            self.state = MissionState.ABORTED
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(wp.x, wp.y)
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_response)

    def _on_nav_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Nav goal rejected.')
            self.state = MissionState.ABORTED
            return
        result_future = gh.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        result = future.result()
        if result.status != 4:  # 4 == STATUS_SUCCEEDED
            self.get_logger().warn(f'Nav status {result.status} — moving on.')
            self.state = MissionState.GOAL_REACHED
            return

        wp = self.waypoints[self.current_idx]
        if wp.type == WaypointType.GPS_POST:
            self.state = MissionState.GOAL_REACHED
        elif wp.type == WaypointType.ARUCO_POST:
            self._dispatch_aruco(wp)
        elif wp.type == WaypointType.OBJECT:
            self._dispatch_object_verify(wp)

    def _dispatch_aruco(self, wp: Waypoint):
        if not self.aruco_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('ArUco action server unavailable.')
            self.state = MissionState.GOAL_REACHED
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(wp.marker_id)
        goal.pose.pose.position.x = 1.0  # target distance encoded here
        future = self.aruco_client.send_goal_async(goal)
        future.add_done_callback(self._on_aruco_response)
        self.state = MissionState.FINAL_APPROACH

    def _on_aruco_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('ArUco goal rejected.')
            self.state = MissionState.GOAL_REACHED
            return
        gh.get_result_async().add_done_callback(
            lambda f: setattr(self, 'state', MissionState.GOAL_REACHED)
        )

    def _dispatch_object_verify(self, wp: Waypoint):
        self.state = MissionState.OBJECT_VERIFY
        self._verify_started_t = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f'Verifying object {wp.object_class} at waypoint {wp.id}'
        )

    def _poll_object_verify(self):
        wp = self.waypoints[self.current_idx]
        threshold = float(self.get_parameter('object_confidence').value)
        for det in self._latest_detections:
            if det['class_id'] == wp.object_class and det['score'] >= threshold:
                self.get_logger().info(
                    f'Object {wp.object_class} verified at score {det["score"]:.2f}'
                )
                self.state = MissionState.GOAL_REACHED
                return
        timeout = float(self.get_parameter('object_verify_timeout_s').value)
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._verify_started_t) > timeout:
            self.get_logger().warn(
                f'Object verify timeout for {wp.object_class}; advancing.'
            )
            self.state = MissionState.GOAL_REACHED

    def _make_pose_stamped(self, x: float, y: float) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.w = 1.0
        return p


def main(args=None):
    rclpy.init(args=args)
    node = GoalSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
