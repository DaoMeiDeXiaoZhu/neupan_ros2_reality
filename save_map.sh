#!/bin/bash
# 保存地图到 maps/ 目录
# 用法: ./save_map.sh [地图名]  不传参则使用 config/frames.yaml 中的 default_map

ROOT=$(cd "$(dirname "$0")" && pwd)

if [ -n "$1" ]; then
    MAP_NAME="$1"
else
    MAP_NAME=$(python3 -c "import yaml; print(yaml.safe_load(open('$ROOT/config/frames.yaml'))['save_map'])")
fi

MAP_DIR="$ROOT/maps"
mkdir -p "$MAP_DIR/$MAP_NAME"

echo "保存地图: $MAP_DIR/$MAP_NAME/my_map"

# 1. 保存地图图像 (.yaml + .pgm)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$MAP_DIR/$MAP_NAME/my_map'}}"

# 2. 保存位姿图序列化文件 (localization 模式需要)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$MAP_DIR/$MAP_NAME/my_map'}"

echo "地图已保存到: $MAP_DIR/$MAP_NAME/"
