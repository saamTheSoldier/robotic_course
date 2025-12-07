import math
import os
import csv
import time
from typing import List, Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile


def yaw_from_quat(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TestNode(Node):
    def __init__(self) -> None:
        super().__init__("test_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("cmd_vel_topic", "/cmd_vel"),
                ("ekf_topic", "/ekf_odometry"),
                ("measurement_topic", "/measurement"),
                ("motion_topic", "/motion_model"),
                ("ground_truth_topic", "/ground_truth/odom"),
                ("linear_speed", 0.2),
                ("angular_speed", 0.5),
                ("edge_time", 2.0),
                ("loops", 1),
                ("csv_dir", "/home/saam/courses/robotics/hw/robotic_course/report/csvs"),
            ],
        )

        qos = QoSProfile(depth=10)
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, qos)

        self.paths: Dict[str, List[PoseStamped]] = {"ekf": [], "measurement": [], "motion": [], "gt": []}
        self.sub_ekf = self.create_subscription(Odometry, self.get_parameter("ekf_topic").value,
                                                lambda msg: self.store_path("ekf", msg), qos)
        self.sub_meas = self.create_subscription(Odometry, self.get_parameter("measurement_topic").value,
                                                 lambda msg: self.store_path("measurement", msg), qos)
        self.sub_motion = self.create_subscription(Odometry, self.get_parameter("motion_topic").value,
                                                   lambda msg: self.store_path("motion", msg), qos)
        self.sub_gt = self.create_subscription(Odometry, self.get_parameter("ground_truth_topic").value,
                                               lambda msg: self.store_path("gt", msg), qos)

        self.path_pubs = {
            "ekf": self.create_publisher(Path, "/ekf_path", qos),
            "measurement": self.create_publisher(Path, "/measurement_path", qos),
            "motion": self.create_publisher(Path, "/motion_path", qos),
            "gt": self.create_publisher(Path, "/real_path", qos),
        }

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.edge_time = float(self.get_parameter("edge_time").value)
        self.loops = int(self.get_parameter("loops").value)

        self.state = "forward"
        self.start_time = time.time()
        self.edge_start = self.start_time
        self.turns_done = 0
        self.timer = self.create_timer(0.05, self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_paths)

        self.csv_dir = self.get_parameter("csv_dir").value
        os.makedirs(self.csv_dir, exist_ok=True)

        self.get_logger().info("test_node running rectangle path")

    def store_path(self, key: str, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.paths[key].append(pose)

    def control_loop(self) -> None:
        t_now = time.time()
        cmd = Twist()
        if self.state == "forward":
            cmd.linear.x = self.linear_speed
            if t_now - self.edge_start >= self.edge_time:
                self.state = "turn"
                self.edge_start = t_now
        elif self.state == "turn":
            cmd.angular.z = self.angular_speed
            if t_now - self.edge_start >= (math.pi / 2) / self.angular_speed:
                self.turns_done += 1
                if self.turns_done >= 4 * self.loops:
                    self.state = "stop"
                else:
                    self.state = "forward"
                self.edge_start = t_now
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            # done: dump and stop timers
            self.timer.cancel()
            self.path_timer.cancel()
            self.dump_csv()
            return

        self.cmd_pub.publish(cmd)

    def publish_paths(self) -> None:
        for key, pub in self.path_pubs.items():
            if not self.paths[key]:
                continue
            path = Path()
            path.header.frame_id = "odom"
            path.poses = self.paths[key]
            pub.publish(path)

    def dump_csv(self) -> None:
        def save(key: str, filename: str):
            path = self.paths[key]
            if not path:
                return
            with open(os.path.join(self.csv_dir, filename), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "x", "y", "theta"])
                for p in path:
                    q = p.pose.orientation
                    theta = yaw_from_quat(q)
                    w.writerow([p.header.stamp.sec + p.header.stamp.nanosec * 1e-9,
                                p.pose.position.x,
                                p.pose.position.y,
                                theta])

        save("gt", "ground_truth.csv")
        save("measurement", "measurement.csv")
        save("motion", "motion.csv")
        save("ekf", "ekf.csv")
        self.get_logger().info("CSV dumps complete")


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


