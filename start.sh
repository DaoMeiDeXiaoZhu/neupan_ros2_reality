#!/bin/bash
# NeuPAN 真机部署一键启动脚本

set -e

ROOT=$(cd "$(dirname "$0")" && pwd)
NEUPAN_ROS2="$ROOT/neupan/neupan_ros2"

# 清除旧缓存，防止加载旧 bytecode
export PYTHONDONTWRITEBYTECODE=1
find "$ROOT" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true

# 加载 neupan Python 包
export PYTHONPATH="$ROOT/neupan/NeuPAN:$PYTHONPATH"

# 激活 conda 环境
echo "=== 1. 激活 conda 环境 ==="
source /home/xiaozhu/anaconda3/etc/profile.d/conda.sh
conda activate env_neupan_ros2

echo "=== 2. 编译 neupan_ros2 工作空间 ==="
cd "$NEUPAN_ROS2"
colcon build --symlink-install

# 修复 ament_python 生成的 shebang，指向 conda 环境 python
sed -i '1s|^#!/usr/bin/python3|#!/usr/bin/env python3|' \
    install/neupan_ros2/lib/neupan_ros2/neupan_node 2>/dev/null || true

echo ""
echo "=== 3. 加载环境 ==="
source install/setup.bash

echo ""
echo "=== 4. 启动 ==="
exec ros2 launch "$ROOT/launch/neupan_navigation.launch.py" "$@"
