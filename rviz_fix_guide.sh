#!/bin/bash

echo "=== RViz瞬移问题修复指南 ==="
echo "专门针对'瞬移只在RViz中出现，Gazebo中正常'的问题"
echo

echo "🎯 问题确认:"
echo "如果机器人在Gazebo中运动正常，但在RViz中显示跳跃，"
echo "说明问题在可视化层面，而不是控制层面。"
echo

echo "✅ 已完成的RViz修复:"
echo "1. 时间戳一致性修复 - 使用ros::Time(0)避免时间戳跳跃"
echo "2. TF查询优化 - 使用稍旧时间戳避免未来时间问题"
echo "3. 降低路径发布频率 - 减少RViz更新负担"
echo "4. 坐标系统一 - 确保所有消息使用'map'坐标系"
echo

echo "🚀 测试步骤:"
echo
echo "步骤1: 运行RViz诊断 (在启动系统前)"
echo "  python3 rviz_diagnostic.py"
echo

echo "步骤2: 启动系统"
echo "  ./start_coverage_system.sh"
echo

echo "步骤3: 在RViz中检查设置"
echo "  - 确认Fixed Frame设置为'map'"
echo "  - 检查Path显示器设置:"
echo "    • Topic: /passedPath"
echo "    • Color: 自定义颜色"
echo "    • Line Width: 0.05"
echo "    • Queue Size: 1"
echo

echo "步骤4: 对比观察"
echo "  - Gazebo窗口: 观察机器人实际运动是否平滑"
echo "  - RViz窗口: 观察路径显示是否还有跳跃"
echo

echo "🔧 如果RViz仍有跳跃，手动调整:"
echo
echo "A. 在RViz中:"
echo "   1. 点击Displays面板中的Path项"
echo "   2. 设置以下参数:"
echo "      - Topic: /passedPath"
echo "      - Buffer Length: 1"
echo "      - Queue Size: 1"
echo "      - Line Style: Lines"
echo "      - Line Width: 0.05-0.1"
echo "      - Alpha: 0.8"
echo

echo "B. 改变Fixed Frame:"
echo "   1. 在Global Options中"
echo "   2. 将Fixed Frame从'odom'改为'map'"
echo "   3. 保存配置"
echo

echo "C. 重置RViz:"
echo "   1. 关闭RViz"
echo "   2. 删除~/.rviz/目录(如果存在)"
echo "   3. 重新启动系统"
echo

echo "📊 验证修复效果:"
echo "修复成功的标志:"
echo "  ✅ Gazebo中机器人运动平滑"
echo "  ✅ RViz中路径显示连续，无跳跃"
echo "  ✅ 机器人轨迹与实际运动一致"
echo "  ✅ 没有大幅度的位置突变"
echo

echo "🎯 根本原因总结:"
echo "RViz瞬移问题通常由以下原因造成:"
echo "1. 时间戳不一致导致TF查询失败"
echo "2. Fixed Frame设置错误"
echo "3. 路径消息发布频率过高"
echo "4. 坐标系变换时的时间戳问题"
echo

echo "现在可以开始测试修复效果！"
echo "建议先运行诊断脚本，然后启动系统观察改进情况。"
