# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2026 Pylox Systems (https://pyloxforge.com)
# Licensed under the PolyForm Noncommercial License 1.0.0.
# See LICENSE or https://polyformproject.org/licenses/noncommercial/1.0.0
# Required Notice: Copyright Pylox Systems (https://pyloxforge.com)

"""ArUco Final-Approach Action Server.

When Nav2 has gotten the rover within ~3 m of a URC GPS-flagged post, this
action takes over and visually servos the final approach using ArUco fiducials
detected by the existing aruco_opencv pipeline (the team's `ros_aruco_opencv`
package, already in their repo).

Action interface (defined inline as Goal/Result/Feedback dataclasses for
simplicity; can be migrated to a proper .action file once integrated):
  Goal:     marker_id (int), approach_distance (float, default 1.0 m)
  Feedback: distance_to_marker (float), heading_error (float)
  Result:   success (bool), final_distance (float), final_heading (float)

Servo law:
  v = clamp(kp_lin * (distance - target_distance), v_min, v_max)
  w = clamp(kp_ang * heading_error, w_min, w_max)
  Stop when |distance - target| < 0.1 and |heading_error| < 0.1 rad for 5 ticks.
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from std_msgs.msg import Header

# aruco_opencv_msgs is the package the team already uses
try:
    from aruco_opencv_msgs.msg import ArucoDetection
except ImportError:
    ArucoDetection = None  # graceful import for non-ArUco builds

# Reuse Nav2's NavigateToPose action signature for compatibility,
# but treat it as a final-approach servo.
from nav2_msgs.action import NavigateToPose


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ArucoFinalApproach(Node):
    def __init__(self):
        super().__init__('aruco_action_server')

        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('detection_topic', '/aruco_detections')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('approach_distance', 1.0)
        self.declare_parameter('servo_kp_linear', 0.3)
        self.declare_parameter('servo_kp_angular', 0.6)
        self.declare_parameter('v_min', 0.05)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('w_min', -0.6)
        self.declare_parameter('w_max', 0.6)
        self.declare_parameter('detection_timeout_s', 2.0)
        self.declare_parameter('lost_marker_search_speed', 0.3)
        self.declare_parameter('settled_iters', 5)

        cb = ReentrantCallbackGroup()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10
        )

        if ArucoDetection is not None:
            self.det_sub = self.create_subscription(
                ArucoDetection,
                self.get_parameter('detection_topic').value,
                self._detection_cb,
                qos,
                callback_group=cb,
            )
        else:
            self.get_logger().warn(
                'aruco_opencv_msgs not found, action server running but will fail '
                'on goal — ensure ros_aruco_opencv package is installed.'
            )
            self.det_sub = None

        self._latest_detection_time: Optional[float] = None
        self._latest_markers = {}  # marker_id -> (distance, bearing)

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            '/aruco_final_approach',
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb,
        )

        self.get_logger().info('ArUco final-approach action server up.')

    def _goal_callback(self, _goal):
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal):
        self._publish_stop()
        return CancelResponse.ACCEPT

    def _detection_cb(self, msg):
        self._latest_detection_time = time.time()
        markers = {}
        for marker in getattr(msg, 'markers', []):
            mid = getattr(marker, 'marker_id', None)
            pose = getattr(marker, 'pose', None)
            if mid is None or pose is None:
                continue
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            distance = math.sqrt(x * x + y * y + z * z)
            # Bearing: angle in horizontal plane from camera optical axis.
            # ZED camera frame: x=right, y=down, z=forward. Bearing = atan2(x, z).
            bearing = math.atan2(x, z)
            markers[mid] = (distance, bearing)
        self._latest_markers = markers

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    async def _execute(self, goal_handle):
        # Reuse NavigateToPose; we encode the marker_id in the goal pose's
        # frame_id field (as a string) and the approach_distance in pose.position.x.
        goal = goal_handle.request.pose
        try:
            marker_id = int(goal.header.frame_id) if goal.header.frame_id else 0
        except ValueError:
            marker_id = 0
        target_distance = (
            goal.pose.position.x
            if goal.pose.position.x > 0.0
            else self.get_parameter('approach_distance').value
        )

        kp_lin = self.get_parameter('servo_kp_linear').value
        kp_ang = self.get_parameter('servo_kp_angular').value
        v_min = self.get_parameter('v_min').value
        v_max = self.get_parameter('v_max').value
        w_min = self.get_parameter('w_min').value
        w_max = self.get_parameter('w_max').value
        det_timeout = self.get_parameter('detection_timeout_s').value
        search_w = self.get_parameter('lost_marker_search_speed').value
        settled_iters_target = int(self.get_parameter('settled_iters').value)

        rate = self.create_rate(20.0)
        settled = 0

        self.get_logger().info(
            f'Approaching marker {marker_id} to {target_distance} m'
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_stop()
                goal_handle.canceled()
                return NavigateToPose.Result()

            now = time.time()
            stale = (
                self._latest_detection_time is None
                or (now - self._latest_detection_time) > det_timeout
            )

            if stale or marker_id not in self._latest_markers:
                # Marker lost — slow rotational search
                cmd = Twist()
                cmd.angular.z = search_w
                self.cmd_pub.publish(cmd)
                settled = 0
                rate.sleep()
                continue

            distance, bearing = self._latest_markers[marker_id]

            distance_err = distance - target_distance
            heading_err = bearing  # 0 = centered

            v = self._clamp(kp_lin * distance_err, -v_max, v_max)
            if 0 < abs(v) < v_min:
                v = math.copysign(v_min, v)
            w = self._clamp(kp_ang * heading_err, w_min, w_max)

            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)

            fb = NavigateToPose.Feedback()
            fb.distance_remaining = float(abs(distance_err))
            goal_handle.publish_feedback(fb)

            if abs(distance_err) < 0.1 and abs(heading_err) < 0.1:
                settled += 1
            else:
                settled = 0

            if settled >= settled_iters_target:
                self._publish_stop()
                goal_handle.succeed()
                self.get_logger().info(
                    f'Marker {marker_id} reached: distance={distance:.2f}, '
                    f'heading_err={heading_err:.2f}'
                )
                return NavigateToPose.Result()

            rate.sleep()

        self._publish_stop()
        goal_handle.abort()
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFinalApproach()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
