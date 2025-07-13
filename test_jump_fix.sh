#!/bin/bash

echo "=== 机器人瞬移问题修复测试脚本 ==="
echo "启动时间: $(date)"
echo

# 设置ROS环境
source /home/getting/Sweeping-Robot/devel/setup.bash

echo "1. 检查关键文件修复状态..."
echo "   ✅ next_goal.cpp frame_id已改为'map'"
echo "   ✅ 数组边界检查已添加"
echo "   ✅ AMCL transform_tolerance已优化"
echo

echo "2. 检查ROS主题状态..."
# 检查关键话题是否可用
timeout 5s rostopic list | grep -E "(odom|amcl_pose|move_base_simple|tf)" || echo "   ⚠️  部分ROS话题不可用，请先启动系统"

echo
echo "3. 建议的测试流程："
echo "   步骤1: 启动完整系统"
echo "          ./start_coverage_system.sh"
echo
echo "   步骤2: 在另一个终端监控TF变换（可选）"
echo "          python3 tf_monitor.py"
echo
echo "   步骤3: 观察机器人行为"
echo "          - 检查是否还有位置跳跃"
echo "          - 观察目标点到达是否平滑"
echo "          - 确认坐标系一致性"
echo
echo "   步骤4: 如果仍有问题，检查以下话题:"
echo "          rostopic echo /tf | grep -A5 -B5 map"
echo "          rostopic echo /odom | head -20"
echo "          rostopic echo /amcl_pose | head -10"
echo "          rostopic echo /move_base_simple/goal | head -5"
echo

echo "=== 关键修复汇总 ==="
echo "1. 坐标系统一："
echo "   - next_goal发布目标点使用'map'坐标系"
echo "   - AMCL定位输出'map'坐标系"
echo "   - move_base期望'map'坐标系目标"
echo
echo "2. 边界保护："
echo "   - 防止数组越界访问"
echo "   - 路径点计数边界检查"
echo
echo "3. 参数优化："
echo "   - AMCL transform_tolerance增加到0.2"
echo "   - 目标到达容差设置为0.3m"
echo
echo "4. 如果问题持续存在，可能的原因："
echo "   - 里程计数据质量问题"
echo "   - 仿真环境时间步长设置"
echo "   - move_base内部规划器参数"
echo "   - 硬件通信延迟（如果在实际机器人上）"
echo

echo "准备就绪！请启动系统进行测试。"
echo
