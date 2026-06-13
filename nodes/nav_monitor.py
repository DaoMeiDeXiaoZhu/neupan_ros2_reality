#!/usr/bin/env python3
"""监控 neupan 导航行为：记录 cmd_vel、goal_pose、arrive/stop 状态。"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import time

class NavMonitor(Node):
    def __init__(self):
        super().__init__("nav_monitor")
        self.start_time = time.time()
        self.last_cmd = None

        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)

        self.get_logger().info("=" * 60)
        self.get_logger().info("导航监控已启动，等待 2D Goal Pose...")
        self.get_logger().info("=" * 60)

    def ts(self):
        return f"T+{time.time() - self.start_time:.2f}s"

    def cmd_cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        key = (round(v, 3), round(w, 3))
        if key != self.last_cmd:
            self.last_cmd = key
            self.get_logger().info(
                f"[{self.ts()}] cmd_vel: v={v:.3f}, w={w:.3f}"
            )

    def goal_cb(self, msg):
        self.get_logger().info(
            f"[{self.ts()}] >>> 收到 Goal: x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f} <<<"
        )

def main():
    rclpy.init()
    rclpy.spin(NavMonitor())

if __name__ == "__main__":
    main()
