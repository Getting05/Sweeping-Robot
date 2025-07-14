#!/bin/bash

# 扫地机器人覆盖率测试脚本
echo "=== 扫地机器人覆盖率监控测试 ==="

# 设置环境
cd /home/getting/Sweeping-Robot
source devel/setup.bash

echo "1. 检查节点是否运行..."
if rosnode list | grep -q "coverage_monitor"; then
    echo "✓ coverage_monitor 节点正在运行"
else
    echo "✗ coverage_monitor 节点未运行"
fi

if rosnode list | grep -q "sweeping_robot_evaluator"; then
    echo "✓ sweeping_robot_evaluator 节点正在运行"
else
    echo "✗ sweeping_robot_evaluator 节点未运行"
fi

echo ""
echo "2. 检查话题状态..."
rostopic list | grep -E "(coverage|passedPath|plan_path)" | while read topic; do
    echo "话题: $topic"
    timeout 3 rostopic hz "$topic" 2>/dev/null | head -1
done

echo ""
echo "3. 检查当前覆盖率..."
echo "正在监听覆盖率话题 (5秒)..."
timeout 5 rostopic echo /coverage_percentage | head -5

echo ""
echo "4. 检查路径信息..."
echo "当前已走过的路径点数:"
timeout 3 rostopic echo /passedPath | grep -o "poses:" | wc -l

echo ""
echo "=== 测试完成 ==="
echo ""
echo "如果覆盖率仍然很低，请检查："
echo "1. 机器人是否在移动"
echo "2. 地图是否正确加载"
echo "3. /passedPath 话题是否有数据"
echo "4. 机器人清扫半径是否合适"
