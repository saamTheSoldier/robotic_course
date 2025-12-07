import math
import os
import csv
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PredictionNode(Node):
    def __init__(self) -> None:
        super().__init__("prediction_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("wheel_radius", 0.1),
                ("wheel_separation", 0.45),
                ("cmd_vel_topic", "/cmd_vel"),
                ("motion_topic", "/motion_model"),
                ("odom_frame", "odom"),
                ("base_frame", "base_link"),
                ("rate", 50.0),
                ("log_csv", "/home/saam/courses/robotics/hw/robotic_course/report/csvs/motion.csv"),
            ],
        )

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.motion_topic = self.get_parameter("motion_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.dt = 1.0 / float(self.get_parameter("rate").value)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.last_cmd: Twist = Twist()

        qos = QoSProfile(depth=10)
        self.cmd_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_cb, qos)
        self.motion_pub = self.create_publisher(Odometry, self.motion_topic, qos)

        self.state = [0.0, 0.0, 0.0]
        self.timer = self.create_timer(self.dt, self.step)

        self.csv_path = self.get_parameter("log_csv").value
        self._csv = None  # type: Optional[csv.writer]
        if self.csv_path:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
            self._csv_file = open(self.csv_path, "w", newline="")
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow(["t", "x", "y", "theta", "v", "omega"])

        self.get_logger().info(f"prediction_node started on {self.motion_topic}")

    def cmd_cb(self, msg: Twist) -> None:
        self.last_cmd = msg

    def step(self) -> None:
        v = self.last_cmd.linear.x
        w = self.last_cmd.angular.z

        x, y, theta = self.state
        x += v * math.cos(theta) * self.dt
        y += v * math.sin(theta) * self.dt
        theta = wrap_angle(theta + w * self.dt)
        self.state = [x, y, theta]

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.motion_pub.publish(odom)

        if self._csv:
            t = self.get_clock().now().nanoseconds * 1e-9
            self._csv.writerow([t, x, y, theta, v, w])

    def destroy_node(self):
        if hasattr(self, "_csv_file"):
            self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PredictionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


