# NeuPAN 真机部署

## 目录结构

```
真机部署NeuPAN/
├── config/
│   └── frames.yaml              # 坐标系和话题名配置
├── urdf/
│   └── test.urdf                # 机器人 TF 树定义
├── nodes/
│   ├── tf_publisher.py           # TF 变换发布（核心节点）
│   ├── lidar.py                  # 雷达过滤节点
│   ├── lidar.yaml                # 雷达过滤参数
│   └── check.py                  # 启动前检查节点
├── launch/
│   ├── slam_mapping.launch.py   # 建图启动
│   └── neupan_navigation.launch.py  # 导航启动
├── rviz/
│   └── neupan_navigation.rviz   # RViz 配置（可选）
├── maps/                        # 保存的地图文件
├── neupan/neupan_ros2/          # NeuPAN 源码（需编译）
├── start.sh                     # 一键编译 + 启动导航
└── save_map.sh                  # 保存地图
```

## 一、部署前必须修改的配置

### 1. config/frames.yaml —— 坐标系和话题

| 字段 | 说明 | 示例 |
|------|------|------|
| `map_frame` | 地图坐标系名，由 slam_toolbox 维护 | `'map'` |
| `odom_frame` | 里程计坐标系名，由底盘驱动发布 | `'odom'` |
| `base_frame` | 机器人本体坐标系名 | `'base_link'` |
| `lidar_frame` | 雷达坐标系名 | `'lidar_frame'` |
| `scan_raw_topic` | 雷达驱动发布的原始话题 | `'/scan_raw'` |
| `scan_topic` | 过滤后的话题，给 slam 和 neupan 用 | `'/scan'` |
| `cmd_vel_topic` | 速度控制话题 | `'/cmd_vel'` |
| `load_map` | 导航时默认加载的地图名 | `'my_map'` |
| `save_map` | 建图后默认保存的地图名 | `'my_map'` |

### 2. urdf/test.urdf —— 机器人 TF 树

定义机器人关节和坐标系变换。至少需要包含 `base_link` 以及雷达相对于 `base_link` 的安装位置：

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link"/>

  <!-- 雷达安装位置 -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_frame"/>
    <origin xyz="0.15 0 0.3" rpy="0 0 0"/>   <!-- 按实际安装位置修改 -->
  </joint>

  <link name="lidar_frame"/>
</robot>
```

> 改完 URDF 后无需重新编译 neupan_ros2，直接启动 launch 即生效。

### 3. neupan/neupan_ros2/src/neupan_ros2/config/robots/<类型>/robot.yaml

NeuPAN 节点参数，**关键字段**：

| 字段 | 说明 |
|------|------|
| `robot_type` | 机器人类型名 |
| `planner_config_file` | 规划器配置文件（相对于本目录） |
| `dune_checkpoint_file` | DUNE 网络权重文件路径 |
| `map_frame` / `base_frame` / `lidar_frame` | 需与 `frames.yaml` 一致 |
| `scan_angle_max` / `scan_angle_min` | 雷达扫描角度范围 |
| `scan_range_max` / `scan_range_min` | 雷达有效距离范围 |
| `control_frequency` | 控制频率 (Hz) |

### 4. neupan/neupan_ros2/src/neupan_ros2/config/robots/<类型>/planner.yaml

规划器参数，**关键字段**：

| 字段 | 说明 |
|------|------|
| `robot.kinematics` | 运动学模型：`'diff'` 差速 / `'omni'` 全向 / `'acker'` 阿克曼 |
| `robot.max_speed` | 最大速度 `[线速度, 角速度]` |
| `robot.max_acce` | 最大加速度 `[线加速度, 角加速度]` |
| `robot.length` / `robot.width` | 机器人外形尺寸 (m) |
| `ref_speed` | 参考巡航速度 (m/s) |
| `adjust.d_min` / `adjust.d_max` | 避障距离范围 (m) |

> 修改 neupan_ros2 下的文件后需重新编译：`./start.sh`

### 5. nodes/lidar.yaml —— 雷达过滤参数

| 字段 | 说明 |
|------|------|
| `input_topic` / `output_topic` | 输入输出话题 |
| `range_min` / `range_max` | 雷达有效距离 (m) |
| `replace_nan` / `replace_inf` | 是否过滤无效值 |

## 二、环境要求

| 步骤 | 环境 | 说明 |
|------|------|------|
| 建图 (`slam_mapping.launch.py`) | **系统环境**（非 conda） | 使用系统自带的 ROS 2 和 slam_toolbox |
| 保存地图 (`save_map.sh`) | **系统环境**（非 conda） | 调用 slam_toolbox 服务保存地图 |
| 导航 (`neupan_navigation.launch.py`) | **conda 环境 `env_neupan_ros2`** | NeuPAN 依赖 torch，需在 conda 环境中运行 |

> **为什么建图和导航要用不同环境？**
> - 建图只需要 slam_toolbox，系统 ROS 2 自带，无需额外依赖
> - 导航需要 NeuPAN 神经网络模块，依赖 torch，而 torch 安装在 conda 环境 `env_neupan_ros2` 中
> - `start.sh` 脚本会自动激活 conda 环境、编译 neupan_ros2 并启动导航

## 三、使用流程

### 第一步：建图（系统环境）

```bash
# 确保当前在系统环境中（不要 activate 任何 conda 环境）
# 如果之前在 conda 环境中，先退出：
conda deactivate

# 启动建图
ros2 launch launch/slam_mapping.launch.py

# 用键盘（弹出的 gnome-terminal）控制小车走一圈
# 建图完成后，新开一个终端（同样是系统环境），运行：
./save_map.sh                    # 保存到 config/frames.yaml 中 save_map 指定的名字
./save_map.sh lab_3f             # 或指定名字
```

> **注意**：建图前将机器人放在起点位置。建图时 `tf_publisher.py` 使用 `use_odom=false`（固定 odom→base_footprint 为原点），确保从 (0,0,0) 开始构建地图。

### 第二步：导航（conda 环境 env_neupan_ros2）

```bash
# 将机器人放回建图时的原点位置和朝向
# start.sh 会自动激活 conda 环境、编译并启动

./start.sh                       # 用 config/frames.yaml 中 load_map 指定的地图
./start.sh map_name:=lab_3f      # 或指定地图名
```

> **注意**：导航时 `tf_publisher.py` 使用 `use_odom=true`（从 /odom 读取真实里程计），启动时自动将里程计位置和朝向清零，确保与地图原点对齐。

## 四、TF 树架构

本项目的 TF 变换由 `nodes/tf_publisher.py` 统一发布（30Hz 到 `/tf`），不再依赖底盘驱动发布 TF：

```
map  ← slam_toolbox 自动维护
  └── odom  ← tf_publisher.py 发布（建图: 固定原点 / 导航: 从 /odom 读取）
        └── base_footprint  ← tf_publisher.py 发布
              └── base_link  ← tf_publisher.py 发布（固定 z=0.07）
                    └── lidar_frame  ← tf_publisher.py 发布（固定安装偏移）
```

- **建图模式** (`use_odom=false`)：odom→base_footprint 发布恒等变换 (0,0,0)，从原点建图
- **导航模式** (`use_odom=true`)：订阅 `/odom` 话题，记录初始值后发布相对偏移量，位置和朝向均做清零处理
- `robot_state_publisher` 仅发布 URDF 中的轮子关节等非固定部件

https://github.com/user-attachments/assets/dabe0751-7034-4145-9969-ccab9f4f0ee1
