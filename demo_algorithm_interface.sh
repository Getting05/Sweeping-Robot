#!/bin/bash

# 路径规划算法接口快速演示脚本

echo "=== 路径规划算法接口快速演示 ==="
echo ""

cd /home/getting/Sweeping-Robot
source devel/setup.bash

echo "✅ 编译状态检查:"
if [ -f "devel/lib/auto_nav/path_planning" ] && [ -f "devel/lib/auto_nav/algorithm_switcher_demo" ]; then
    echo "   - path_planning: 已编译 ✓"
    echo "   - algorithm_switcher_demo: 已编译 ✓"
else
    echo "   - 正在编译项目..."
    catkin_make > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "   - 编译成功 ✓"
    else
        echo "   - 编译失败 ✗"
        exit 1
    fi
fi

echo ""
echo "✅ 关键文件检查:"
files=(
    "src/auto_nav/include/path_planning_interface.h"
    "src/auto_nav/src/path_planning_interface.cpp"
    "src/auto_nav/config/path_planning_algorithms.yaml"
    "PATH_PLANNING_INTERFACE_GUIDE.md"
    "ALGORITHM_INTERFACE_SUMMARY.md"
)

for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo "   - $file ✓"
    else
        echo "   - $file ✗"
    fi
done

echo ""
echo "🎯 算法接口功能演示:"
echo ""
echo "1. 支持的算法类型:"
echo "   • Neural Network - 原有神经网络算法 (默认)"
echo "   • A* Algorithm   - 启发式搜索算法"
echo "   • D* Algorithm   - 动态环境规划算法"
echo "   • MCP Snake      - 蛇形覆盖算法"
echo "   • MCP Spiral     - 螺旋覆盖算法"
echo "   • MCP Zone       - 分区覆盖算法"

echo ""
echo "2. 算法切换方式:"
echo "   方式1: 配置文件 - 编辑 src/auto_nav/config/path_planning_algorithms.yaml"
echo "   方式2: ROS参数 - rosparam set /algorithm_type 'astar'"
echo "   方式3: 话题消息 - rostopic pub /switch_algorithm std_msgs/String \"data: 'astar'\""
echo "   方式4: 智能菜单 - ./start_intelligent_cleaning.sh (选择算法管理)"

echo ""
echo "3. 当前配置查看:"
if [ -f "src/auto_nav/config/path_planning_algorithms.yaml" ]; then
    echo "   当前算法类型: $(grep "algorithm_type:" src/auto_nav/config/path_planning_algorithms.yaml | awk '{print $2}' | tr -d '\"')"
    echo "   A*权重参数: $(grep "heuristic_weight:" src/auto_nav/config/path_planning_algorithms.yaml | awk '{print $2}')"
    echo "   覆盖模式: $(grep "coverage_pattern:" src/auto_nav/config/path_planning_algorithms.yaml | awk '{print $2}')"
fi

echo ""
echo "🚀 快速使用指南:"
echo ""
echo "启动完整系统 (推荐):"
echo "  ./start_intelligent_cleaning.sh"
echo "  选择 '1) 智能启动' 或 '7) 算法管理'"
echo ""
echo "命令行快速切换:"
echo "  # 启动ROS核心 (在第一个终端)"
echo "  roscore"
echo ""
echo "  # 启动路径规划节点 (在第二个终端)"
echo "  source devel/setup.bash"
echo "  roslaunch auto_nav sequential_clean.launch"
echo ""
echo "  # 切换算法 (在第三个终端)"
echo "  source devel/setup.bash"
echo "  rostopic pub /switch_algorithm std_msgs/String \"data: 'astar'\" --once"
echo ""
echo "查看详细文档:"
echo "  cat PATH_PLANNING_INTERFACE_GUIDE.md"
echo "  cat ALGORITHM_INTERFACE_SUMMARY.md"

echo ""
echo "=== 演示完成 ==="
echo ""
echo "💡 提示: 运行 './start_intelligent_cleaning.sh' 体验完整的算法切换功能！"
