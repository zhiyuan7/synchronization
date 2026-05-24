#!/bin/bash

# 脚本用于设置 ROS2 工作空间的便捷别名

BASHRC="$HOME/.bashrc"
MARKER="# >>> ROS2 Vision Workspace Aliases >>> "

# 定义所有别名
declare -A ALIASES=(
    ["cb-export-compile-commands"]="colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash"
    ["cbs-export-compile-commands"]="colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash"
    ["cb-release"]="colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash"
    ["cb-release-with-debug-info"]="colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo && source install/setup.bash"
    ["cbs-release"]="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash"
    ["cbs-release-export-compile-commands"]="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash"
    ["cb"]="colcon build && source install/setup.bash"
    ["cbs"]="colcon build --symlink-install && source install/setup.bash"
    ["cbp"]="colcon build --packages-select"
    ["si"]="source install/setup.bash"
    ["ccw"]="colcon clean workspace"
    ["ccp"]="colcon clean packages --packages-select"
    ["jc"]="sudo jetson_clocks --fan"
    ["nm"]="sudo systemctl stop gdm3 && sudo /etc/NX/nxserver --restart"
    ["lt"]="ros2 launch track track.launch.py"
    ["lp"]="ros2 launch pos pos.launch.py"
    ["ld"]="ros2 launch detect detect.launch.py"
    ["lf"]="ros2 launch filter filter.launch.py"
    ["lvf"]="ros2 launch filter vi_filter.launch.py"
    ["lvp"]="ros2 launch pos vi_pos.launch.py"
    ["lvd"]="ros2 launch detect vi_detect.launch.py"
    ["lvt"]="ros2 launch track vi_track.launch.py"
    ["lva"]="ros2 launch all vi_all.launch.py"
    ["ic"]="ros2 run image_transport republish raw compressed --ros-args --remap in:=/image_raw --remap out/compressed:=/image_compressed"
    ["ri"]="ros2 bag record --max-cache-size 2147483648 /image_compressed /detect/detections /stm32"
    ["rd"]="ros2 bag record /detect/detections /stm32"
)

# 检查每个别名是否已存在
declare -a MISSING_ALIASES=()
declare -a EXISTING_ALIASES=()

for alias_name in "${!ALIASES[@]}"; do
    if grep -q "alias ${alias_name}=" "$BASHRC" 2>/dev/null; then
        EXISTING_ALIASES+=("$alias_name")
    else
        MISSING_ALIASES+=("$alias_name")
    fi
done

# 显示状态
echo "================================================"
echo "ROS2 工作空间别名状态检查"
echo "================================================"
echo ""

if [ ${#EXISTING_ALIASES[@]} -gt 0 ]; then
    echo "✓ 已存在的别名："
    for alias_name in "${EXISTING_ALIASES[@]}"; do
        echo "  ${alias_name} -> ${ALIASES[$alias_name]}"
    done
    echo ""
fi

if [ ${#MISSING_ALIASES[@]} -eq 0 ]; then
    echo "✓ 所有别名都已添加，无需操作"
    exit 0
fi

echo "✗ 缺少的别名："
for alias_name in "${MISSING_ALIASES[@]}"; do
    echo "  ${alias_name} -> ${ALIASES[$alias_name]}"
done
echo ""

# 询问是否添加缺少的别名
read -p "是否添加缺少的别名? (y/n): " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "取消操作"
    exit 0
fi

# 检查是否需要添加标记
if ! grep -q "$MARKER" "$BASHRC" 2>/dev/null; then
    echo "" >> "$BASHRC"
    echo "$MARKER" >> "$BASHRC"
fi

# 添加缺少的别名
for alias_name in "${MISSING_ALIASES[@]}"; do
    echo "alias ${alias_name}='${ALIASES[$alias_name]}'" >> "$BASHRC"
    echo "✓ 已添加: ${alias_name}"
done

echo ""
echo "================================================"
echo "别名添加完成！"
echo "================================================"
echo ""
echo "请运行以下命令使别名生效："
echo "  source ~/.bashrc"
echo ""
echo "或者关闭并重新打开终端"
