#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseArray


class CubeChaseController(Node):
    """
    Minimal visual-servo controller:
    - Sub:  /cube_poses (PoseArray)   # positions are in camera frame (X,Y,Z) meters
    - Pub:  /cmd_vel   (Twist)

    Logic:
    - Choose nearest cube (min Z)
    - Angular control: omega = -k_w * X   (turn to reduce lateral offset)
    - Linear control:  v = k_v * (Z - target_z)
    """

    def __init__(self):
        super().__init__("cube_chase_controller")

        # ===== Params (tune on-site) =====
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("pose_topic", "/cube_poses")

        self.declare_parameter("enable", True)

        # target distance to stop in front of cube
        self.declare_parameter("target_z", 0.60)          # meters
        self.declare_parameter("z_stop_deadband", 0.08)   # meters

        # gains
        self.declare_parameter("k_v", 0.8)   # linear gain
        self.declare_parameter("k_w", 2.0)   # angular gain

        # limits
        self.declare_parameter("v_max", 0.35)      # m/s
        self.declare_parameter("w_max", 1.2)       # rad/s

        # safety
        self.declare_parameter("timeout_sec", 0.5)  # if no detection, stop
        self.declare_parameter("publish_rate_hz", 20.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.enable = bool(self.get_parameter("enable").value)

        self.target_z = float(self.get_parameter("target_z").value)
        self.z_stop_deadband = float(self.get_parameter("z_stop_deadband").value)

        self.k_v = float(self.get_parameter("k_v").value)
        self.k_w = float(self.get_parameter("k_w").value)

        self.v_max = float(self.get_parameter("v_max").value)
        self.w_max = float(self.get_parameter("w_max").value)

        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        # ===== ROS I/O =====
        self.sub = self.create_subscription(PoseArray, self.pose_topic, self.on_poses, 10)
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.last_msg_time: Optional[float] = None
        self.latest_poses: Optional[PoseArray] = None

        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(
            f"CubeChaseController started. Sub={self.pose_topic} Pub={self.cmd_vel_topic}"
        )

    def on_poses(self, msg: PoseArray):
        self.latest_poses = msg
        self.last_msg_time = self.get_clock().now().nanoseconds * 1e-9

    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # if disabled -> stop
        if not self.enable:
            self.publish_stop()
            return

        # timeout -> stop
        if self.last_msg_time is None or (now - self.last_msg_time) > self.timeout_sec:
            self.publish_stop()
            return

        if self.latest_poses is None or len(self.latest_poses.poses) == 0:
            self.publish_stop()
            return

        # choose nearest cube (min Z)
        best = min(self.latest_poses.poses, key=lambda p: p.position.z)

        X = float(best.position.x)  # left/right in camera frame
        Z = float(best.position.z)  # forward distance

        # angular: turn to reduce X (sign might need flipping on-site)
        w = -self.k_w * X

        # linear: approach target_z
        z_err = Z - self.target_z
        if abs(z_err) < self.z_stop_deadband:
            v = 0.0
        else:
            v = self.k_v * z_err

        # clamp
        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))

        # publish
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub.publish(cmd)

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = CubeChaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()