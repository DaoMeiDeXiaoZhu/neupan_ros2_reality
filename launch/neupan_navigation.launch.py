import sys
sys.dont_write_bytecode = True

import os
import yaml
import subprocess
import tempfile

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    # 将 package:// 替换为 file://（与 view_mentorpi.sh 完全一致）
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


def create_tf_publisher(use_odom=False):
    """发布 odom→base_footprint→base_link→lidar_frame TF 链。

    use_odom=False: odom→base_footprint 固定 (0,0,0)，适合建图
    use_odom=True:  从 /odom 话题读取真实里程计，适合导航
    """
    script = os.path.join(pkg_root, 'nodes', 'tf_publisher.py')
    cmd = ['python3', script]
    if use_odom:
        cmd += ['--ros-args', '-p', 'use_odom:=true']
    return ExecuteProcess(
        cmd=cmd,
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


def create_slam_localization_node(map_file):
    """SLAM 纯定位节点：加载已有地图，只定位不建图。"""
    slam_params = {
        'scan_topic': frames['scan_topic'],
        'odom_frame': frames['odom_frame'],
        'map_frame': frames['map_frame'],
        'base_frame': frames['base_frame'],
        'map_update_interval': 2.0,
        'resolution': 0.05,
        'minimum_travel_distance': 0.0,
        'minimum_travel_heading': 0.0,
        'loop_search_max_distance': 3.0,
        'loop_match_min_chain_size': 10,
        'mode': 'localization',
        'map_file_name': map_file,
        'map_start_pose': [0.0, 0.0, 0.0],
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


def create_neupan_node(robot_config_dir):
    """NeuPAN 规划节点：加载 neupan_ros2 内置的 robot.yaml。"""
    robot_yaml = os.path.join(robot_config_dir, 'robot.yaml')
    return Node(
        package='neupan_ros2',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_yaml,
            {'robot_config_dir': robot_config_dir},
        ],
        remappings=[
            ('/neupan_cmd_vel', frames['cmd_vel_topic']),
        ],
    )


def create_check_node():
    """启动检查节点：验证必备话题和 TF 链是否就绪。"""
    script = os.path.join(pkg_root, 'nodes', 'check.py')
    return ExecuteProcess(
        cmd=['python3', script],
        name='startup_check',
        output='screen',
    )


def create_rviz_node():
    """RViz2 可视化节点：有配置则加载，没有则空启动。"""
    rviz_config = os.path.join(pkg_root, 'rviz', 'neupan_navigation.rviz')
    args = ['-d', rviz_config] if os.path.exists(rviz_config) else []
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=args,
        output='screen',
    )


def generate_launch_description():
    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value=frames['load_map'],
        description='地图名（对应 maps/ 下的子目录）',
    )
    robot_type_arg = DeclareLaunchArgument(
        'robot_type', default_value='scout',
        description='机器人类型（scout/limo/ranger/simulation）',
    )

    robot_desc = generate_robot_description()

    tf_pub = create_tf_publisher(use_odom=True)
    robot_state_pub = create_robot_state_publisher(robot_desc)
    joint_state_pub = create_joint_state_publisher()
    lidar_node = create_lidar_node()

    map_name = LaunchConfiguration('map_name')
    robot_type = LaunchConfiguration('robot_type')

    from ament_index_python.packages import get_package_share_directory
    neupan_pkg_share = get_package_share_directory('neupan_ros2')
    robot_config_dir = os.path.join(neupan_pkg_share, 'config', 'robots', 'scout')

    map_file = os.path.join(pkg_root, 'maps', frames['load_map'], 'my_map')
    slam_node = TimerAction(
        period=2.0,
        actions=[create_slam_localization_node(map_file)],
    )

    neupan_node = TimerAction(
        period=4.0,
        actions=[create_neupan_node(robot_config_dir)],
    )

    check_node = TimerAction(
        period=6.0,
        actions=[create_check_node()],
    )

    rviz_node = create_rviz_node()

    return LaunchDescription([
        map_name_arg,
        robot_type_arg,
        tf_pub,
        robot_state_pub,
        joint_state_pub,
        lidar_node,
        slam_node,
        neupan_node,
        check_node,
        rviz_node,
    ])
