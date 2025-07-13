#!/bin/bash

# 清扫机器人覆盖率监控系统启动脚本
# 使用方法：./start_coverage_system.sh [仿真|真实]

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的信息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查ROS环境
check_ros_environment() {
    print_info "检查ROS环境..."
    
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS环境未设置，请先source ROS setup.bash"
        exit 1
    fi
    
    print_success "ROS环境检查通过 (ROS $ROS_DISTRO)"
}

# 检查工作空间
check_workspace() {
    print_info "检查工作空间..."
    
    if [ ! -f "devel/setup.bash" ]; then
        print_error "请在ROS工作空间根目录运行此脚本"
        exit 1
    fi
    
    if [ ! -f "src/auto_nav/scripts/coverage_monitor.py" ]; then
        print_error "coverage_monitor.py脚本不存在"
        exit 1
    fi
    
    print_success "工作空间检查通过"
}

# 编译工作空间
build_workspace() {
    print_info "编译工作空间..."
    
    if catkin_make > /dev/null 2>&1; then
        print_success "编译成功"
    else
        print_error "编译失败，请检查代码"
        exit 1
    fi
}

# 显示使用方法
show_usage() {
    echo "清扫机器人覆盖率监控系统启动脚本"
    echo ""
    echo "使用方法："
    echo "  $0 [模式]"
    echo ""
    echo "模式选项："
    echo "  sim    - 仿真模式（包含Gazebo仿真环境）"
    echo "  real   - 真实机器人模式"
    echo "  viz    - 仅可视化模式（不启动导航）"
    echo "  test   - 系统测试模式"
    echo ""
    echo "示例："
    echo "  $0 sim    # 启动仿真模式"
    echo "  $0 real   # 启动真实机器人模式"
    echo "  $0 test   # 运行系统测试"
}

# 启动仿真模式
start_simulation() {
    print_info "启动仿真模式..."
    print_info "将启动以下组件："
    print_info "  - Gazebo仿真环境"
    print_info "  - RViz可视化"
    print_info "  - 导航系统"
    print_info "  - 覆盖率监控"
    print_info "  - 自动清扫节点"
    
    print_warning "请确保没有其他ROS节点在运行，特别是Gazebo和RViz"
    read -p "按Enter键继续，或Ctrl+C取消..."
    
    source devel/setup.bash
    roslaunch auto_nav sequential_clean_with_coverage.launch
}

# 启动真实机器人模式
start_real_robot() {
    print_info "启动真实机器人模式..."
    print_info "将启动以下组件："
    print_info "  - RViz可视化"
    print_info "  - 导航系统"
    print_info "  - 覆盖率监控"
    print_info "  - 自动清扫节点"
    
    print_warning "请确保："
    print_warning "  1. 机器人硬件已连接并正常工作"
    print_warning "  2. 传感器数据正常发布"
    print_warning "  3. 地图已加载"
    
    read -p "按Enter键继续，或Ctrl+C取消..."
    
    source devel/setup.bash
    roslaunch auto_nav sequential_clean_with_coverage_viz.launch
}

# 启动可视化模式
start_visualization() {
    print_info "启动可视化模式..."
    print_info "仅启动RViz和覆盖率监控"
    
    source devel/setup.bash
    roslaunch auto_nav sequential_clean_with_coverage_viz.launch
}

# 运行系统测试
run_system_test() {
    print_info "运行系统测试..."
    print_info "这将检查所有核心话题是否正常工作"
    
    source devel/setup.bash
    
    # 检查roscore是否运行
    if ! pgrep -f roscore > /dev/null; then
        print_warning "ROS master未运行，正在启动..."
        roscore &
        sleep 3
    fi
    
    # 运行测试
    python3 src/auto_nav/scripts/system_test.py 30
}

# 主函数
main() {
    clear
    print_info "清扫机器人覆盖率监控系统"
    print_info "=================================="
    
    # 检查环境
    check_ros_environment
    check_workspace
    
    # 解析参数
    MODE=${1:-""}
    
    case $MODE in
        "sim")
            build_workspace
            start_simulation
            ;;
        "real")
            build_workspace
            start_real_robot
            ;;
        "viz")
            build_workspace
            start_visualization
            ;;
        "test")
            build_workspace
            run_system_test
            ;;
        "help"|"-h"|"--help")
            show_usage
            ;;
        "")
            print_error "请指定运行模式"
            echo ""
            show_usage
            exit 1
            ;;
        *)
            print_error "未知模式: $MODE"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

# 信号处理
trap 'print_info "用户中断，正在清理..."; exit 0' INT TERM

# 运行主函数
main "$@"
