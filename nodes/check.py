#!/usr/bin/env python3
"""
启动前检查节点：验证必备话题和 TF 链是否就绪。
"""

import time
import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import LaserScan


class StartupCheck(Node):
    """检查必备话题和 TF，缺哪个打印哪个。"""

    def __init__(self):
        super().__init__("startup_check")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.scan_ok = False
        self.map_odom_ok = False
        self.odom_base_ok = False
        self.base_lidar_ok = False

    def check_scan(self, timeout=3.0):
        """等待 /scan 话题有发布者。"""
        topic_list = self.get_topic_names_and_types()
        topics = {t for t, _ in topic_list}
        self.get_logger().info("检查话题 /scan ...")
        t0 = time.time()
        while "/scan" not in topics:
            if time.time() - t0 > timeout:
                self.get_logger().error("话题 /scan 不存在，请检查雷达驱动是否启动")
                return False
            time.sleep(0.5)
            topic_list = self.get_topic_names_and_types()
            topics = {t for t, _ in topic_list}
        self.scan_ok = True
        self.get_logger().info("话题 /scan 就绪")
        return True

    def check_tf_chain(self, source, target, label, timeout=3.0):
        """检查 TF 从 source 到 target 是否存在。"""
        self.get_logger().info(f"检查 TF: {source} → {target} ...")
        t0 = time.time()
        while True:
            try:
                self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
                self.get_logger().info(f"TF {source} → {target} 就绪")
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
                if time.time() - t0 > timeout:
                    self.get_logger().error(f"TF {source} → {target} 超时未找到: {e}")
                    return False
                time.sleep(0.5)

    def run_all_checks(self):
        """按依赖顺序执行所有检查。"""
        self.get_logger().info("========== 启动检查开始 ==========")

        # 1. 话题检查
        self.check_scan()

        # 2. TF 链检查（由下往上）
        self.odom_base_ok = self.check_tf_chain("base_link", "odom", "odom → base_link")
        self.base_lidar_ok = self.check_tf_chain("lidar_frame", "base_link", "base_link → lidar_frame")
        self.map_odom_ok = self.check_tf_chain("odom", "map", "map → odom")

        # 汇总
        all_ok = all([self.scan_ok, self.map_odom_ok, self.odom_base_ok, self.base_lidar_ok])

        if all_ok:
            self.get_logger().info("========== 所有检查通过 ==========")
        else:
            self.get_logger().error("========== 以下检查失败 ==========")
            if not self.scan_ok:
                self.get_logger().error("  /scan 话题缺失")
            if not self.odom_base_ok:
                self.get_logger().error("  TF odom → base_link 缺失（请检查里程计）")
            if not self.base_lidar_ok:
                self.get_logger().error("  TF base_link → lidar 缺失（请检查雷达外参或 URDF）")
            if not self.map_odom_ok:
                self.get_logger().error("  TF map → odom 缺失（请启动 slam_toolbox）")

        return all_ok


def main(args=None):
    rclpy.init(args=args)
    node = StartupCheck()
    ok = node.run_all_checks()
    node.destroy_node()
    rclpy.shutdown()
    exit(0 if ok else 1)


if __name__ == "__main__":
    main()
