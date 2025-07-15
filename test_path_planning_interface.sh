#!/bin/bash

# 路径规划算法接口测试脚本
# 此脚本用于测试不同算法的切换功能

echo "=== 路径规划算法接口测试 ==="

# 编译项目
echo "1. 编译项目..."
cd /home/getting/Sweeping-Robot
catkin_make

if [ $? -eq 0 ]; then
    echo "✓ 编译成功"
else
    echo "✗ 编译失败"
    exit 1
fi

# 源环境
source devel/setup.bash

echo ""
echo "2. 可用的算法接口测试命令:"
echo ""
echo "# 启动路径规划节点 (在单独终端中运行)"
echo "roslaunch auto_nav sequential_clean.launch"
echo ""
echo "# 查看当前算法状态"
echo "rostopic echo /path_planning/status"
echo ""
echo "# 切换到不同算法:"
echo "rostopic pub /switch_algorithm std_msgs/String \"data: 'neural_network'\""
echo "rostopic pub /switch_algorithm std_msgs/String \"data: 'astar'\""
echo "rostopic pub /switch_algorithm std_msgs/String \"data: 'dstar'\""
echo "rostopic pub /switch_algorithm std_msgs/String \"data: 'mcp'\""
echo ""
echo "# 查看算法参数"
echo "rosparam get /algorithm_type"
echo "rosparam get /heuristic_weight"
echo "rosparam get /coverage_pattern"
echo ""

# 检查文件是否存在
echo "3. 检查关键文件:"
files=(
    "src/auto_nav/include/path_planning_interface.h"
    "src/auto_nav/src/path_planning_interface.cpp"
    "src/auto_nav/config/path_planning_algorithms.yaml"
    "PATH_PLANNING_INTERFACE_GUIDE.md"
)

for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo "✓ $file"
    else
        echo "✗ $file (不存在)"
    fi
done

echo ""
echo "4. 算法切换示例:"
echo ""
echo "# 在一个终端中启动系统："
echo "cd /home/getting/Sweeping-Robot"
echo "source devel/setup.bash"
echo "./start_intelligent_cleaning.sh smart"
echo ""
echo "# 在另一个终端中测试算法切换："
echo "source devel/setup.bash"
echo ""
echo "# 切换到A*算法"
echo "rostopic pub /switch_algorithm std_msgs/String \"data: 'astar'\" --once"
echo ""
echo "# 切换到MCP蛇形覆盖模式"
echo "rosparam set /coverage_pattern 0"
echo "rostopic pub /switch_algorithm std_msgs/String \"data: 'mcp'\" --once"
echo ""
echo "# 切换到MCP螺旋覆盖模式"
echo "rosparam set /coverage_pattern 1"
echo "rostopic pub /switch_algorithm std_msgs/String \"data: 'mcp'\" --once"
echo ""

echo "=== 测试完成 ==="
echo ""
echo "详细使用说明请参考: PATH_PLANNING_INTERFACE_GUIDE.md"
echo ""
echo "快速开始:"
echo "1. 运行: ./start_intelligent_cleaning.sh smart"
echo "2. 在另一个终端切换算法"
echo "3. 观察路径规划的变化"
