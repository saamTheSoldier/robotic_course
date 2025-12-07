import math
import os
import csv
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile


def yaw_from_quat(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MeasurementNode(Node):
    def __init__(self) -> None:
        super().__init__("measurement_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("vo_topic", "/vo/odom"),
                ("imu_topic", "/zed/zed_node/imu/data_raw"),
                ("output_topic", "/measurement"),
                ("odom_frame", "odom"),
                ("base_frame", "base_link"),
                ("log_csv", "/home/saam/courses/robotics/hw/robotic_course/report/csvs/measurement.csv"),
            ],
        )

        qos = QoSProfile(depth=10)
        self.vo_sub = self.create_subscription(
            Odometry, self.get_parameter("vo_topic").value, self.vo_cb, qos
        )
        self.imu_sub = self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self.imu_cb, qos
        )

        self.pub = self.create_publisher(Odometry, self.get_parameter("output_topic").value, qos)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.last_vo: Optional[Odometry] = None
        self.last_theta: Optional[float] = None

        self.csv_path = self.get_parameter("log_csv").value
        self._csv = None  # type: Optional[csv.writer]
        if self.csv_path:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
            self._csv_file = open(self.csv_path, "w", newline="")
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow(["t", "x", "y", "theta"])

        self.get_logger().info("measurement_node ready")

    def imu_cb(self, msg: Imu) -> None:
        self.last_theta = yaw_from_quat(msg.orientation)
        self.try_publish()

    def vo_cb(self, msg: Odometry) -> None:
        self.last_vo = msg
        self.try_publish()

    def try_publish(self) -> None:
        if self.last_vo is None or self.last_theta is None:
            return

        x = self.last_vo.pose.pose.position.x
        y = self.last_vo.pose.pose.position.y
        theta = self.last_theta

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.pub.publish(odom)

        if self._csv:
            t = self.get_clock().now().nanoseconds * 1e-9
            self._csv.writerow([t, x, y, theta])

    def destroy_node(self):
        if hasattr(self, "_csv_file"):
            self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MeasurementNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


