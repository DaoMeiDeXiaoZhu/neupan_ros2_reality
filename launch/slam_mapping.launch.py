import sys
sys.dont_write_bytecode = True

import os
import yaml
import subprocess
import tempfile

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


# 项目根目录
pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# URDF 源目录（与 view_mentorpi.sh 保持一致）
URDF_SRC_DIR = '/home/xiaozhu/桌面/幻尔小车URDF'
CHASSIS = 'mecanum'

# 加载坐标系配置
with open(os.path.join(pkg_root, 'config', 'frames.yaml'), 'r') as f:
    frames = yaml.safe_load(f)


def generate_robot_description():
    """动态处理 xacro 文件生成 URDF 字符串。

    完全复用 view_mentorpi.sh 的管线：
    1. 创建包含底盘和惯性矩阵引用的 xacro 输入
    2. 运行 xacro 生成纯 URDF
    3. 将 package:// 替换为 file:// 绝对路径
    """
    xacro_main = os.path.join(URDF_SRC_DIR, 'urdf', f'{CHASSIS}.xacro')
    inertial = os.path.join(URDF_SRC_DIR, 'urdf', 'inertial_matrix.xacro')

    xacro_input = (
        '<?xml version="1.0" encoding="utf-8"?>\n'
        '<robot name="mentorpi" xmlns:xacro="http://ros.org/wiki/xacro">\n'
        f'    <xacro:include filename="{xacro_main}"/>\n'
        f'    <xacro:include filename="{inertial}"/>\n'
        '</robot>'
    )

    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.xacro', delete=False
    ) as f:
        f.write(xacro_input)
        tmp_xacro = f.name

    try:
        result = subprocess.run(
            ['xacro', tmp_xacro],
            capture_output=True, text=True, check=True,
        )
        urdf = result.stdout
    finally:
        os.unlink(tmp_xacro)

    urdf = urdf.replace(
        'package://mentorpi_description', f'file://{URDF_SRC_DIR}'
    )

    return urdf


def create_robot_state_publisher(robot_desc):
    """从 URDF 发布机器人 TF（需 /joint_states 触发）。"""
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )


def create_joint_state_publisher():
    """发布默认 /joint_states，触发 robot_state_publisher 发布完整 TF 树。"""
    return Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )


def create_tf_publisher():
    """高频发布固定 TF 链到 /tf 上。

    不用 static_transform_publisher（发 /tf_static），因为 slam_toolbox
    内部的 TF buffer 在启动早期经常收不到 /tf_static，导致 message_filter
    无法解析 lidar_frame → odom 链，所有扫描被丢弃。直接在 /tf 上以 30Hz
    发布整条固定链，彻底绕过这个时序问题。
    """
    script = os.path.join(pkg_root, 'nodes', 'tf_publisher.py')
    return ExecuteProcess(
        cmd=['python3', script],
        name='fixed_tf_publisher',
        output='screen',
    )


def create_lidar_node():
    """雷达过滤节点：订阅 /scan_raw，过滤无效值后发布 /scan。"""
    script = os.path.join(pkg_root, 'nodes', 'lidar.py')
    return ExecuteProcess(
        cmd=['python3', script],
        name='lidar_filter',
        output='screen',
    )


def create_slam_mapping_node():
    """SLAM 在线建图节点。"""
    slam_params = {
        'scan_topic': frames['scan_topic'],
        'odom_frame': frames['odom_frame'],
        'map_frame': frames['map_frame'],
        'base_frame': frames['base_frame'],
        'map_update_interval': 2.0,
        'resolution': 0.05,
        # 允许处理静止帧（odom 是静态的，机器人不移动也建图）
        'minimum_travel_distance': 0.0,
        'minimum_travel_heading': 0.0,
        'loop_search_max_distance': 3.0,
        'loop_match_min_chain_size': 10,
        'mode': 'mapping',
        # TF 查找超时，默认 0.2s 太短，静态 TF 可能来不及进 buffer
        'transform_timeout': 1.0,
        'tf_buffer_duration': 30.0,
    }
    return Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )


def create_teleop_node():
    """键盘控制节点：发布 /cmd_vel，控制小车移动。"""
    return ExecuteProcess(
        cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        name='teleop_keyboard',
        output='screen',
        prefix='gnome-terminal --',
    )


def create_rviz_node():
    """RViz2 可视化节点：加载预配置的显示项。"""
    rviz_config = os.path.join(pkg_root, 'rviz', 'slam_mapping.rviz')
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )


def create_check_node():
    """启动检查节点：验证必备话题和 TF 链是否就绪。"""
    script = os.path.join(pkg_root, 'nodes', 'check.py')
    return ExecuteProcess(
        cmd=['python3', script],
        name='startup_check',
        output='screen',
    )


def generate_launch_description():
    # 生成一次 URDF，共享给 robot_state_publisher 和 joint_state_publisher
    robot_desc = generate_robot_description()

    tf_pub = create_tf_publisher()
    robot_state_pub = create_robot_state_publisher(robot_desc)
    joint_state_pub = create_joint_state_publisher()
    lidar_node = create_lidar_node()

    slam_node = TimerAction(
        period=2.0,
        actions=[create_slam_mapping_node()],
    )

    check_node = TimerAction(
        period=6.0,
        actions=[create_check_node()],
    )

    teleop_node = create_teleop_node()
    rviz_node = create_rviz_node()

    return LaunchDescription([
        tf_pub,
        robot_state_pub,
        joint_state_pub,
        lidar_node,
        slam_node,
        check_node,
        teleop_node,
        rviz_node,
    ])
