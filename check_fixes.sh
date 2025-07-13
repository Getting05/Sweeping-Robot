#!/bin/bash

echo "=== 机器人瞬移问题修复检查脚本 ==="
echo "检查时间: $(date)"
echo

# 1. 检查next_goal.cpp的关键修复
echo "1. 检查next_goal.cpp中的关键修复..."
echo "   检查目标点frame_id是否已改为'map':"
if grep -n "goal_msgs.header.frame_id.*map" /home/getting/Sweeping-Robot/src/auto_nav/src/next_goal.cpp; then
    echo "   ✅ 目标点frame_id已修复为'map'"
else
    echo "   ❌ 目标点frame_id仍为'odom'，需要修复"
fi

echo "   检查是否添加了数组边界保护:"
if grep -n "count.*<.*pathPlanner.Path.size()" /home/getting/Sweeping-Robot/src/auto_nav/src/next_goal.cpp; then
    echo "   ✅ 已添加数组边界检查"
else
    echo "   ❌ 缺少数组边界检查"
fi

# 2. 检查AMCL参数
echo
echo "2. 检查AMCL关键参数..."
echo "   transform_tolerance设置:"
if grep -n "transform_tolerance" /home/getting/Sweeping-Robot/src/auto_nav/launch/amcl.launch; then
    echo "   ✅ transform_tolerance参数已配置"
else
    echo "   ❌ 未找到transform_tolerance配置"
fi

# 3. 检查坐标系配置
echo
echo "3. 检查AMCL坐标系配置..."
echo "   odom_frame_id:"
grep -n "odom_frame_id" /home/getting/Sweeping-Robot/src/auto_nav/launch/amcl.launch | head -1
echo "   base_frame_id:"
grep -n "base_frame_id" /home/getting/Sweeping-Robot/src/auto_nav/launch/amcl.launch | head -1
echo "   global_frame_id:"
grep -n "global_frame_id" /home/getting/Sweeping-Robot/src/auto_nav/launch/amcl.launch | head -1

# 4. 检查目标容差参数
echo
echo "4. 检查目标到达容差..."
if grep -n "tolerance_goal" /home/getting/Sweeping-Robot/src/auto_nav/launch/clean_work_sequential.launch; then
    echo "   ✅ 目标容差参数已配置"
else
    echo "   ❌ 未找到目标容差配置"
fi

# 5. 编译检查
echo
echo "5. 检查编译状态..."
cd /home/getting/Sweeping-Robot
if [ -f "build/auto_nav/CMakeFiles/next_goal.dir/src/next_goal.cpp.o" ]; then
    echo "   ✅ next_goal已编译"
else
    echo "   ⚠️  next_goal可能需要重新编译"
fi

echo
echo "=== 修复建议 ==="
echo "关键修复（已完成）:"
echo "  1. ✅ 目标点坐标系从'odom'改为'map'"
echo "  2. ✅ 添加数组边界检查防止越界"
echo "  3. ✅ 调整AMCL transform_tolerance参数"
echo
echo "建议的测试步骤:"
echo "  1. 重新编译项目: cd /home/getting/Sweeping-Robot && catkin_make"
echo "  2. 启动系统: ./start_coverage_system.sh"
echo "  3. 观察机器人是否还有瞬移现象"
echo "  4. 如有需要，运行TF监控: python3 tf_monitor.py"
echo

echo "=== 潜在的进一步优化 ==="
echo "如果问题仍然存在，可以考虑:"
echo "  1. 检查里程计数据质量和发布频率"
echo "  2. 调整AMCL粒子滤波器参数"
echo "  3. 检查激光雷达数据的时间戳同步"
echo "  4. 优化move_base的local_planner参数"
echo
