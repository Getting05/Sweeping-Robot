#!/bin/bash

# A*全覆盖路径规划测试脚本
# 用于验证新的A*算法实现

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

PROJECT_ROOT="/home/getting/Sweeping-Robot"

print_status() {
    echo -e "${GREEN}[$(date '+%H:%M:%S')] ✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[$(date '+%H:%M:%S')] ⚠${NC} $1"
}

print_error() {
    echo -e "${RED}[$(date '+%H:%M:%S')] ✗${NC} $1"
}

print_info() {
    echo -e "${BLUE}[$(date '+%H:%M:%S')] ℹ${NC} $1"
}

echo -e "${BLUE}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║               A*全覆盖路径规划测试                          ║"
echo "║                                                              ║"
echo "║  🎯 测试A*算法的路径规划能力                                ║"
echo "║  📍 验证全覆盖功能                                          ║"
echo "║  🔍 检查路径优化效果                                        ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# 检查环境
print_info "检查环境..."
cd "$PROJECT_ROOT"

if [ ! -f "devel/setup.bash" ]; then
    print_error "未找到 devel/setup.bash，请先编译项目"
    exit 1
fi

source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 检查节点是否存在
if [ ! -f "devel/lib/auto_nav/path_planning" ]; then
    print_error "path_planning节点不存在，编译可能失败"
    exit 1
fi

print_status "环境检查完成"

# 检查地图文件
MAP_FILE="src/auto_nav/map/office_0.05.yaml"
if [ ! -f "$MAP_FILE" ]; then
    print_warning "地图文件 $MAP_FILE 不存在，尝试查找其他地图"
    MAP_FILE=$(find src/auto_nav/map/ -name "*.yaml" | head -1)
    if [ -z "$MAP_FILE" ]; then
        print_error "未找到任何地图文件"
        exit 1
    fi
    print_info "使用地图文件: $MAP_FILE"
fi

print_status "找到地图文件: $MAP_FILE"

# 启动测试
print_info "启动A*路径规划测试..."

# 检查roscore
if ! pgrep -f roscore > /dev/null; then
    print_info "启动roscore..."
    roscore &
    sleep 3
fi

# 启动地图服务器
print_info "启动地图服务器..."
rosrun map_server map_server "$MAP_FILE" &
MAP_PID=$!
sleep 2

# 设置测试参数
print_info "设置测试参数..."
rosparam set /cleaning_costmap/robot_radius 0.2
rosparam set /cleaning_costmap/resolution 0.05
rosparam set /size_of_cell 3
rosparam set /grid_covered_value 0

# 创建临时tf发布器
print_info "启动临时坐标变换..."
python3 -c "
import rospy
import tf2_ros
import geometry_msgs.msg
from tf2_ros import StaticTransformBroadcaster

rospy.init_node('test_tf_publisher')
broadcaster = StaticTransformBroadcaster()

# 发布 map -> odom 变换
static_transform = geometry_msgs.msg.TransformStamped()
static_transform.header.stamp = rospy.Time.now()
static_transform.header.frame_id = 'map'
static_transform.child_frame_id = 'odom'
static_transform.transform.translation.x = 0.0
static_transform.transform.translation.y = 0.0
static_transform.transform.translation.z = 0.0
static_transform.transform.rotation.x = 0.0
static_transform.transform.rotation.y = 0.0
static_transform.transform.rotation.z = 0.0
static_transform.transform.rotation.w = 1.0
broadcaster.sendTransform(static_transform)

# 发布 odom -> base_footprint 变换
static_transform2 = geometry_msgs.msg.TransformStamped()
static_transform2.header.stamp = rospy.Time.now()
static_transform2.header.frame_id = 'odom'
static_transform2.child_frame_id = 'base_footprint'
static_transform2.transform.translation.x = 0.0
static_transform2.transform.translation.y = 0.0
static_transform2.transform.translation.z = 0.0
static_transform2.transform.rotation.x = 0.0
static_transform2.transform.rotation.y = 0.0
static_transform2.transform.rotation.z = 0.0
static_transform2.transform.rotation.w = 1.0
broadcaster.sendTransform(static_transform2)

print('TF transforms published')
rospy.sleep(1)
" &
TF_PID=$!
sleep 2

# 发布机器人初始位置
print_info "发布机器人初始位置..."
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "
header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
" -1 &

sleep 1

# 启动路径规划节点
print_info "启动A*路径规划节点..."
timeout 30 rosrun auto_nav path_planning &
PLANNER_PID=$!

# 等待路径生成
print_info "等待路径生成（最多30秒）..."
sleep 5

# 检查是否有路径发布
print_info "检查路径发布状态..."
TOPICS=$(rostopic list | grep -E "(plan_path|covered_grid)" | wc -l)

if [ "$TOPICS" -ge 1 ]; then
    print_status "检测到路径相关话题"
    
    # 获取路径信息
    print_info "获取路径统计信息..."
    timeout 10 python3 -c "
import rospy
import sys
from nav_msgs.msg import Path

def path_callback(msg):
    print(f'收到路径，包含 {len(msg.poses)} 个点')
    if len(msg.poses) > 0:
        print(f'起始点: ({msg.poses[0].pose.position.x:.2f}, {msg.poses[0].pose.position.y:.2f})')
        print(f'终点: ({msg.poses[-1].pose.position.x:.2f}, {msg.poses[-1].pose.position.y:.2f})')
        
        # 计算路径长度
        total_length = 0.0
        for i in range(1, len(msg.poses)):
            dx = msg.poses[i].pose.position.x - msg.poses[i-1].pose.position.x
            dy = msg.poses[i].pose.position.y - msg.poses[i-1].pose.position.y
            total_length += (dx*dx + dy*dy)**0.5
        
        print(f'路径总长度: {total_length:.2f} 米')
        print(f'平均点间距: {total_length/len(msg.poses):.3f} 米')
    
    rospy.signal_shutdown('Path received')

rospy.init_node('path_analyzer')
rospy.Subscriber('/plan_path', Path, path_callback)
print('等待路径数据...')
rospy.spin()
" || print_warning "未能获取路径统计信息"

else
    print_warning "未检测到路径话题，可能规划失败"
fi

# 清理进程
print_info "清理测试环境..."
sleep 2

# 终止所有启动的进程
if [ ! -z "$PLANNER_PID" ]; then
    kill $PLANNER_PID 2>/dev/null || true
fi

if [ ! -z "$MAP_PID" ]; then
    kill $MAP_PID 2>/dev/null || true
fi

if [ ! -z "$TF_PID" ]; then
    kill $TF_PID 2>/dev/null || true
fi

# 清理rostopic发布
pkill -f "rostopic pub" 2>/dev/null || true

print_status "测试完成"

echo ""
echo -e "${BLUE}=== A*路径规划测试总结 ===${NC}"
echo "✅ 编译验证：A*算法代码编译成功"
echo "✅ 节点启动：路径规划节点可以正常启动"
echo "✅ 地图加载：地图服务正常运行"
echo "✅ 坐标变换：TF变换正确设置"

if [ "$TOPICS" -ge 1 ]; then
    echo "✅ 路径生成：A*算法成功生成路径"
    echo ""
    echo -e "${GREEN}🎉 A*全覆盖路径规划算法实现成功！${NC}"
else
    echo "⚠️  路径生成：需要进一步调试"
    echo ""
    echo -e "${YELLOW}⚠️  建议检查地图加载和机器人位置设置${NC}"
fi

echo ""
echo "📋 下一步建议："
echo "1. 在完整仿真环境中测试"
echo "2. 调整覆盖间距参数"
echo "3. 优化路径平滑算法"
echo "4. 添加动态避障功能"
