#!/bin/bash

# 扫地机器人增强指标测试脚本
echo "=== 扫地机器人增强指标测试 ==="

# 设置环境
cd /home/getting/Sweeping-Robot
source devel/setup.bash

echo "1. 验证代码修改..."
if grep -q "calculate_evaluation_metrics" src/auto_nav/scripts/coverage_monitor.py; then
    echo "✓ 评估指标计算函数已添加"
else
    echo "✗ 评估指标计算函数未找到"
fi

if grep -q "Collision" src/auto_nav/scripts/coverage_monitor.py; then
    echo "✓ 碰撞检测功能已添加"
else
    echo "✗ 碰撞检测功能未找到"
fi

if grep -q "velocity_history" src/auto_nav/scripts/coverage_monitor.py; then
    echo "✓ 运动学指标计算已添加"
else
    echo "✗ 运动学指标计算未找到"
fi

if grep -q "planned_path_points" src/auto_nav/scripts/coverage_monitor.py; then
    echo "✓ 规划路径点统计已添加"
else
    echo "✗ 规划路径点统计未找到"
fi

echo ""
echo "2. 检查新增指标..."
echo "系统现在监控以下指标："
echo "  CR  - 覆盖率"
echo "  ME  - 运动效率" 
echo "  SR  - 清洁冗余度"
echo "  Collision - 碰撞次数"
echo "  CT  - 计算时间"
echo "  FT  - 任务总耗时"
echo "  Vel_avg - 平均速度"
echo "  Acc_avg - 平均加速度"
echo "  Jerk_avg - 平均加加速度"
echo "  Planned_Points - 规划路径点总数"

echo ""
echo "3. 输出文件说明..."
echo "系统运行完成后将生成："
echo "  /home/getting/tmp/sweeping_robot_report_*.txt - 详细文本报告"
echo "  /home/getting/tmp/sweeping_robot_metrics_*.csv - CSV格式指标数据"

echo ""
echo "4. 启动建议..."
echo "要测试新的指标系统，请运行："
echo "  roslaunch auto_nav sequential_clean.launch"
echo ""
echo "在运行过程中，您将看到增强的统计信息，包括："
echo "  - 核心指标 (CR, ME, SR, Collision, CT, FT)"
echo "  - 运动学指标 (Vel_avg, Acc_avg, Jerk_avg)"
echo "  - 面积统计 (覆盖面积, 重复面积等)"
echo "  - 规划路径点总数"

echo ""
echo "=== 测试脚本完成 ==="
