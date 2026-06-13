#!/usr/bin/env python3
"""
TF 发布节点：发布 odom→base_footprint→base_link→lidar_frame 变换链。

- use_odom=false: odom→base_footprint 固定 (0,0,0)，适合建图
- use_odom=true:  odom→base_footprint 从 /odom 话题读取真实里程计，适合导航
"""
import os, yaml, rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__("odom_tf_publisher")
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        with open(os.path.join(pkg_root, "config", "frames.yaml"), "r") as f:
            self.frames = yaml.safe_load(f)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.use_odom = self.declare_parameter("use_odom", False).value

        # 里程计位姿（当前值）
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0
        self.odom_qx = 0.0
        self.odom_qy = 0.0
        self.odom_qz = 0.0
        self.odom_qw = 1.0

        # 启动时的初始里程计值（用于清零位置和朝向）
        self.init_odom_x = None
        self.init_odom_y = None
        self.init_odom_z = None
        self.init_odom_qx = None
        self.init_odom_qy = None
        self.init_odom_qz = None
        self.init_odom_qw = None

        if self.use_odom:
            self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        rate = self.declare_parameter("publish_rate", 30.0).value
        self.timer = self.create_timer(1.0 / rate, self.publish_transforms)
        self.get_logger().info(
            f"TF 发布器启动，{rate}Hz，use_odom={self.use_odom}"
        )

    def odom_cb(self, msg: Odometry):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        if self.init_odom_x is None:
            self.init_odom_x = msg.pose.pose.position.x
            self.init_odom_y = msg.pose.pose.position.y
            self.init_odom_z = msg.pose.pose.position.z
            self.init_odom_qx = qx
            self.init_odom_qy = qy
            self.init_odom_qz = qz
            self.init_odom_qw = qw
            self.get_logger().info(
                f"里程计清零: pos=({self.init_odom_x:.3f}, {self.init_odom_y:.3f}, {self.init_odom_z:.3f})"
            )

        # 位置：相对于启动时的偏移
        self.odom_x = msg.pose.pose.position.x - self.init_odom_x
        self.odom_y = msg.pose.pose.position.y - self.init_odom_y
        self.odom_z = msg.pose.pose.position.z - self.init_odom_z

        # 朝向：相对旋转 q_rel = q_init^{-1} * q_curr
        # q_init^{-1} = (-qx_i, -qy_i, -qz_i, qw_i)
        iqx, iqy, iqz, iqw = (
            -self.init_odom_qx, -self.init_odom_qy,
            -self.init_odom_qz,  self.init_odom_qw,
        )
        self.odom_qx = iqw * qx + iqx * qw + iqy * qz - iqz * qy
        self.odom_qy = iqw * qy - iqx * qz + iqy * qw + iqz * qx
        self.odom_qz = iqw * qz + iqx * qy - iqy * qx + iqz * qw
        self.odom_qw = iqw * qw - iqx * qx - iqy * qy - iqz * qz

    def publish_transforms(self):
        now = self.get_clock().now().to_msg()
        tfs = []

        # 1. odom → base_footprint
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.frames["odom_frame"]
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = self.odom_x
        tf.transform.translation.y = self.odom_y
        tf.transform.translation.z = self.odom_z
        tf.transform.rotation.x = self.odom_qx
        tf.transform.rotation.y = self.odom_qy
        tf.transform.rotation.z = self.odom_qz
        tf.transform.rotation.w = self.odom_qw
        tfs.append(tf)

        # 2. base_footprint → base_link
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "base_footprint"
        tf.child_frame_id = "base_link"
        tf.transform.translation.z = 0.07
        tf.transform.rotation.w = 1.0
        tfs.append(tf)

        # 3. base_link → lidar_frame
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "base_link"
        tf.child_frame_id = self.frames["lidar_frame"]
        tf.transform.translation.x = -0.012242
        tf.transform.translation.y = -8.533e-05
        tf.transform.translation.z = 0.092501
        tf.transform.rotation.w = 1.0
        tfs.append(tf)

        self.tf_broadcaster.sendTransform(tfs)


def main():
    rclpy.init()
    node = OdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
