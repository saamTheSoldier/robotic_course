import math
import os
import csv
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from rclpy.qos import QoSProfile


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class EkfNode(Node):
    def __init__(self) -> None:
        super().__init__("ekf_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("wheel_radius", 0.1),
                ("wheel_separation", 0.45),
                ("motion_topic", "/motion_model"),
                ("measurement_topic", "/measurement"),
                ("ekf_topic", "/ekf_odometry"),
                ("odom_frame", "odom"),
                ("base_frame", "base_link"),
                ("q_noise", [0.05, 0.05]),
                ("r_noise", [0.1, 0.1, 0.05]),
                ("p0", [0.05, 0.05, 0.02]),
                ("log_csv", "/home/saam/courses/robotics/hw/robotic_course/report/csvs/ekf.csv"),
            ],
        )

        qos = QoSProfile(depth=20)
        self.motion_sub = self.create_subscription(
            Odometry, self.get_parameter("motion_topic").value, self.motion_cb, qos
        )
        self.measurement_sub = self.create_subscription(
            Odometry, self.get_parameter("measurement_topic").value, self.meas_cb, qos
        )
        self.pub = self.create_publisher(Odometry, self.get_parameter("ekf_topic").value, qos)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        p0 = self.get_parameter("p0").value
        self.x = np.zeros((3, 1))
        self.P = np.diag([p0[0] ** 2, p0[1] ** 2, p0[2] ** 2])

        q = self.get_parameter("q_noise").value
        self.Q = np.diag([q[0] ** 2, q[1] ** 2])
        r = self.get_parameter("r_noise").value
        self.R = np.diag([r[0] ** 2, r[1] ** 2, r[2] ** 2])
        self.last_motion_stamp = None  # type: Optional[rclpy.time.Time]

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.csv_path = self.get_parameter("log_csv").value
        self._csv = None  # type: Optional[csv.writer]
        if self.csv_path:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
            self._csv_file = open(self.csv_path, "w", newline="")
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow(["t", "x", "y", "theta"])

        self.get_logger().info("ekf_node started")

    def motion_cb(self, msg: Odometry) -> None:
        now = self.get_clock().now()
        if self.last_motion_stamp is None:
            self.last_motion_stamp = now
            return
        dt = (now - self.last_motion_stamp).nanoseconds * 1e-9
        self.last_motion_stamp = now

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        theta = self.x[2, 0]

        # prediction
        Fx = np.array(
            [
                [1.0, 0.0, -v * dt * math.sin(theta)],
                [0.0, 1.0, v * dt * math.cos(theta)],
                [0.0, 0.0, 1.0],
            ]
        )
        B = np.array(
            [
                [math.cos(theta) * dt, 0.0],
                [math.sin(theta) * dt, 0.0],
                [0.0, dt],
            ]
        )

        self.x[0, 0] += v * math.cos(theta) * dt
        self.x[1, 0] += v * math.sin(theta) * dt
        self.x[2, 0] = wrap_angle(self.x[2, 0] + w * dt)

        self.P = Fx @ self.P @ Fx.T + B @ self.Q @ B.T
        self.publish(msg.header.stamp)

    def meas_cb(self, msg: Odometry) -> None:
        z = np.array(
            [
                [msg.pose.pose.position.x],
                [msg.pose.pose.position.y],
                [math.atan2(2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z),
                            1 - 2 * (msg.pose.pose.orientation.z ** 2))],
            ]
        )
        # measurement model h(x) = x
        H = np.identity(3)
        y = z - H @ self.x
        y[2, 0] = wrap_angle(y[2, 0])
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2, 0] = wrap_angle(self.x[2, 0])
        I = np.identity(3)
        self.P = (I - K @ H) @ self.P

        self.publish(msg.header.stamp)

    def publish(self, stamp) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x[0, 0])
        odom.pose.pose.position.y = float(self.x[1, 0])
        odom.pose.pose.orientation.z = math.sin(self.x[2, 0] / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.x[2, 0] / 2.0)
        self.pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = odom.pose.pose.orientation
        self.broadcaster.sendTransform(tf_msg)

        if self._csv:
            t = self.get_clock().now().nanoseconds * 1e-9
            self._csv.writerow([t, self.x[0, 0], self.x[1, 0], self.x[2, 0]])

    def destroy_node(self):
        if hasattr(self, "_csv_file"):
            self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EkfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


