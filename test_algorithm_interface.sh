#!/bin/bash

echo "=== 路径规划算法接口测试 ==="

# 设置ROS环境
source /home/getting/Sweeping-Robot/devel/setup.bash

echo "1. 启动测试A*算法..."
timeout 10s roslaunch auto_nav sequential_clean_with_algorithms.launch algorithm:=astar &
LAUNCH_PID=$!

sleep 5

echo "2. 测试算法切换服务..."
# 检查服务是否可用
if rosservice list | grep -q "/path_planning/set_algorithm"; then
    echo "✓ 算法切换服务可用"
    
    # 测试切换到神经网络算法
    echo "3. 切换到神经网络算法..."
    rosservice call /path_planning/set_algorithm "algorithm_type: 'neural'"
    
    echo "4. 设置算法参数..."
    rosservice call /path_planning/set_parameter "param_name: 'c_0' param_value: 60.0"
    
    echo "5. 切换回A*算法..."
    rosservice call /path_planning/set_algorithm "algorithm_type: 'astar'"
    
    echo "✓ 算法接口测试完成"
else
    echo "✗ 算法切换服务不可用"
fi

# 清理
echo "6. 清理测试环境..."
kill $LAUNCH_PID 2>/dev/null
sleep 2
killall -9 roscore rosmaster 2>/dev/null

echo "=== 测试完成 ==="
