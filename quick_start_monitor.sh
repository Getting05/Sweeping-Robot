#!/bin/bash

# 清扫机器人启动和监控脚本

echo "🤖 清扫机器人覆盖率监控系统"
echo "=================================="

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS环境未设置，请先运行: source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 检查工作空间
if [ ! -f "devel/setup.bash" ]; then
    echo "❌ 请在ROS工作空间根目录运行此脚本"
    exit 1
fi

echo "✅ 环境检查通过"
echo ""

# 编译工作空间
echo "🔧 编译工作空间..."
if catkin_make > /dev/null 2>&1; then
    echo "✅ 编译成功"
else
    echo "❌ 编译失败"
    exit 1
fi

echo ""
echo "🚀 启动选项:"
echo "1. 启动清扫系统（包含覆盖率监控）"
echo "2. 仅启动覆盖率监控终端"
echo ""

read -p "请选择选项 (1/2): " choice

case $choice in
    1)
        echo ""
        echo "🎯 启动完整清扫系统..."
        echo "系统将启动以下组件："
        echo "  - Gazebo仿真环境"
        echo "  - RViz可视化界面"  
        echo "  - 导航和路径规划"
        echo "  - 自动清扫节点"
        echo "  - 覆盖率监控节点"
        echo ""
        echo "📝 使用说明："
        echo "  1. 等待系统完全启动"
        echo "  2. 在RViz中添加以下显示项："
        echo "     - Path: /cleaned_path (红色，已清扫路径)"
        echo "     - Path: /move_base/NavfnROS/plan (绿色，当前规划)"
        echo "     - Map: /covered_area_grid (覆盖区域)"
        echo "  3. 在新终端运行监控: python3 src/auto_nav/scripts/monitor_coverage.py"
        echo ""
        read -p "按Enter键继续..."
        
        source devel/setup.bash
        roslaunch auto_nav sequential_clean.launch
        ;;
    2)
        echo ""
        echo "📊 启动覆盖率监控终端..."
        echo "请确保清扫系统已在其他终端启动"
        echo ""
        
        source devel/setup.bash
        python3 src/auto_nav/scripts/monitor_coverage.py
        ;;
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac
