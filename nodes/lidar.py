#!/usr/bin/env python3
"""
激光雷达过滤节点：订阅 /scan_raw，修正 NaN/Inf/越界值后发布 /scan。
"""

import os
import math
import yaml

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan


def load_config(node):
    """从节点所在目录的 lidar.yaml 加载配置。"""
    config_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(config_dir, "lidar.yaml")
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


def build_qos(config):
    """根据 yaml 配置构建 QoSProfile。"""
    reliability_map = {
        "reliable": QoSReliabilityPolicy.RELIABLE,
        "best_effort": QoSReliabilityPolicy.BEST_EFFORT,
    }
    durability_map = {
        "volatile": QoSDurabilityPolicy.VOLATILE,
        "transient_local": QoSDurabilityPolicy.TRANSIENT_LOCAL,
    }
    reliability = reliability_map.get(
        config.get("qos_reliability", "best_effort"),
        QoSReliabilityPolicy.BEST_EFFORT,
    )
    durability = durability_map.get(
        config.get("qos_durability", "volatile"),
        QoSDurabilityPolicy.VOLATILE,
    )
    qos = QoSProfile(depth=config.get("queue_size", 10))
    qos.reliability = reliability
    qos.durability = durability
    return qos


def filter_scan(scan_msg, config):
    """过滤单帧激光雷达数据：NaN/Inf → range_max，截断越界值。"""
    ranges = list(scan_msg.ranges)
    range_min = config.get("range_min", 0.05)
    range_max = config.get("range_max", 30.0)
    replace_nan = config.get("replace_nan", True)
    replace_inf = config.get("replace_inf", True)
    clip = config.get("clip_range", True)

    for i, r in enumerate(ranges):
        if replace_nan and math.isnan(r):
            ranges[i] = range_max
        elif replace_inf and math.isinf(r):
            ranges[i] = range_max
        elif r <= 0.0:
            ranges[i] = range_max
        elif r > range_max:
            ranges[i] = range_max if clip else r
        elif r < range_min:
            ranges[i] = range_min if clip else r

    scan_msg.ranges = ranges
    return scan_msg


def resample_scan(scan_msg, target_n):
    """将 scan 数据均匀重采样到 target_n 个点，保持角度范围不变。"""
    if target_n <= 0:
        return scan_msg
    n = len(scan_msg.ranges)
    if n == target_n:
        return scan_msg

    old_angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, n)
    new_angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, target_n)
    scan_msg.ranges = np.interp(new_angles, old_angles, scan_msg.ranges).tolist()
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (target_n - 1) if target_n > 1 else 0.0
    return scan_msg


class LidarFilter(Node):
    """ROS2 节点：订阅原始雷达数据，过滤后重新发布。"""

    def __init__(self):
        super().__init__("lidar_filter")

        # 加载配置
        config = load_config(self)
        self.config = config

        # QoS
        qos = build_qos(config)

        # 发布
        output_topic = config.get("output_topic", "/scan")
        self.publisher = self.create_publisher(LaserScan, output_topic, qos)

        # 订阅
        input_topic = config.get("input_topic", "/scan_raw")
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, qos
        )

        self.publish_rate = config.get("publish_rate", 0.0)
        self.resample_points = config.get("resample_points", 0)

        # 如果配置了固定频率，用 timer 驱动发布
        if self.publish_rate > 0:
            period = 1.0 / self.publish_rate
            self.latest_scan = None
            self.create_timer(period, self.timer_callback)
            self.get_logger().info(
                f"定时发布模式，频率 {self.publish_rate} Hz"
            )

        self.get_logger().info(
            f"LidarFilter 已启动: {input_topic} → {output_topic}"
        )

    def scan_callback(self, msg):
        """收到原始雷达数据，过滤后发布。"""
        filtered = filter_scan(msg, self.config)
        filtered = resample_scan(filtered, self.resample_points)

        if self.publish_rate > 0:
            # 定时模式：缓存最新一帧，由 timer 发布
            self.latest_scan = filtered
        else:
            # 直通模式：来一帧发一帧
            self.publisher.publish(filtered)

    def timer_callback(self):
        """定时发布最新一帧数据。"""
        if self.latest_scan is not None:
            filter_scan(self.latest_scan, self.config)
            resample_scan(self.latest_scan, self.resample_points)
            self.latest_scan.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.latest_scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
