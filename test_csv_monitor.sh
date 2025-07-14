#!/bin/bash

# 扫地机器人实时CSV数据功能测试脚本
# 用于验证和启动coverage_monitor.py的新功能

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目根目录
PROJECT_ROOT="/home/getting/Sweeping-Robot"
MONITOR_SCRIPT="$PROJECT_ROOT/src/auto_nav/scripts/coverage_monitor.py"

echo -e "${BLUE}=== 扫地机器人实时CSV数据功能测试 ===${NC}"

# 函数：打印状态信息
print_status() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# 检查文件存在性
check_files() {
    echo -e "${BLUE}检查必要文件...${NC}"
    
    if [ -f "$MONITOR_SCRIPT" ]; then
        print_status "找到监控脚本: $MONITOR_SCRIPT"
    else
        print_error "监控脚本不存在: $MONITOR_SCRIPT"
        exit 1
    fi
    
    # 检查脚本内容
    if grep -q "csv_save_interval = 30.0" "$MONITOR_SCRIPT"; then
        print_status "发现CSV自动保存功能配置"
    else
        print_error "CSV自动保存功能未配置"
        exit 1
    fi
    
    if grep -q "save_realtime_csv_data" "$MONITOR_SCRIPT"; then
        print_status "发现CSV保存方法"
    else
        print_error "CSV保存方法未实现"
        exit 1
    fi
}

# 检查ROS环境
check_ros_environment() {
    echo -e "${BLUE}检查ROS环境...${NC}"
    
    if [ -z "$ROS_DISTRO" ]; then
        print_warning "ROS环境变量未设置，尝试source setup文件"
        if [ -f "$PROJECT_ROOT/devel/setup.bash" ]; then
            source "$PROJECT_ROOT/devel/setup.bash"
            print_status "已source项目setup文件"
        else
            print_error "找不到setup文件，请先编译项目"
            exit 1
        fi
    else
        print_status "ROS环境已配置: $ROS_DISTRO"
    fi
    
    # 检查roscore是否运行
    if pgrep -f roscore > /dev/null; then
        print_status "roscore已运行"
    else
        print_warning "roscore未运行，建议先启动roscore"
    fi
}

# 检查目录权限
check_permissions() {
    echo -e "${BLUE}检查权限...${NC}"
    
    if [ -w "/home/getting/tmp" ]; then
        print_status "/home/getting/tmp目录可写，CSV文件将保存到此目录"
    else
        print_error "/home/getting/tmp目录不可写"
        exit 1
    fi
}

# 显示功能信息
show_feature_info() {
    echo -e "${BLUE}=== 新增功能特性 ===${NC}"
    echo "1. 每30秒自动保存一次实时数据到CSV"
    echo "2. 包含20个核心评估指标"
    echo "3. 支持追加模式，数据持续累积"
    echo "4. 文件名包含时间戳，避免冲突"
    echo "5. 保存位置: /home/getting/tmp/sweeping_robot_realtime_data_<timestamp>.csv"
    echo ""
}

# 显示CSV字段说明
show_csv_fields() {
    echo -e "${BLUE}=== CSV数据字段说明 ===${NC}"
    cat << EOF
字段名                    | 说明                | 单位
-------------------------|--------------------|---------
Timestamp                | 保存时间戳          | -
Runtime_s                | 运行时间            | 秒
Runtime_min              | 运行时间            | 分钟
Coverage_Rate            | 覆盖率              | 0-1
Motion_Efficiency        | 运动效率            | m/m²
Redundancy               | 冗余度              | 0-1
Collision_Count          | 碰撞次数            | 次
Avg_Computation_Time     | 平均计算时间        | 秒
Total_Time               | 总耗时              | 秒
Avg_Velocity             | 平均速度            | m/s
Avg_Acceleration         | 平均加速度          | m/s²
Avg_Jerk                 | 平均加加速度        | m/s³
Planned_Points           | 规划路径点数        | 个
Path_Length              | 路径长度            | 米
Covered_Area             | 已覆盖面积          | m²
Redundant_Area           | 重复面积            | m²
Free_Area_Total          | 总自由面积          | m²
Path_Points_Count        | 路径点数量          | 个
Completed_Goals          | 完成目标数          | 个
Goal_Progress_Rate       | 目标完成率          | %
EOF
    echo ""
}

# 启动选项菜单
show_menu() {
    echo -e "${BLUE}=== 启动选项 ===${NC}"
    echo "1) 仅启动覆盖监控 (需要手动启动清扫任务)"
    echo "2) 启动完整清扫系统 (覆盖监控 + sequential_clean)"
    echo "3) 启动演示系统 (覆盖监控 + demo_system)"
    echo "4) 监控现有CSV文件"
    echo "5) 分析CSV数据 (需要Python pandas)"
    echo "6) 显示使用说明"
    echo "0) 退出"
    echo ""
}

# 监控CSV文件
monitor_csv_files() {
    echo -e "${BLUE}监控CSV文件生成...${NC}"
    echo "按 Ctrl+C 停止监控"
    echo ""
    
    while true; do
        clear
        echo -e "${BLUE}=== 实时CSV文件监控 ===${NC}"
        echo "时间: $(date)"
        echo ""
        
        # 查找CSV文件
        csv_files=($(ls /home/getting/tmp/sweeping_robot_realtime_data_*.csv 2>/dev/null || true))
        
        if [ ${#csv_files[@]} -eq 0 ]; then
            print_warning "未找到CSV文件，等待生成..."
        else
            for file in "${csv_files[@]}"; do
                echo "文件: $file"
                echo "大小: $(du -h "$file" | cut -f1)"
                echo "行数: $(wc -l < "$file")"
                echo "最后修改: $(stat -c %y "$file")"
                
                if [ -s "$file" ]; then
                    echo "最新数据:"
                    tail -n 1 "$file" | cut -d',' -f1-6
                fi
                echo "---"
            done
        fi
        
        sleep 5
    done
}

# 启动覆盖监控
start_coverage_monitor() {
    echo -e "${BLUE}启动覆盖监控...${NC}"
    
    cd "$PROJECT_ROOT"
    source devel/setup.bash
    
    echo "启动 coverage_monitor.py..."
    echo "CSV文件将保存到: /home/getting/tmp/sweeping_robot_realtime_data_<timestamp>.csv"
    echo "按 Ctrl+C 停止监控"
    echo ""
    
    rosrun auto_nav coverage_monitor.py
}

# 启动完整系统
start_full_system() {
    echo -e "${BLUE}启动完整清扫系统...${NC}"
    
    # 检查地图参数
    map_name="${1:-hospital_0.1}"
    
    cd "$PROJECT_ROOT"
    source devel/setup.bash
    
    echo "启动系统组件..."
    echo "地图: $map_name"
    echo "CSV文件: /home/getting/tmp/sweeping_robot_realtime_data_<timestamp>.csv"
    echo ""
    
    # 启动覆盖监控在后台
    rosrun auto_nav coverage_monitor.py &
    monitor_pid=$!
    
    # 等待一下让监控启动
    sleep 2
    
    # 启动清扫任务
    echo "启动清扫任务..."
    roslaunch auto_nav sequential_clean.launch map_name:="$map_name"
    
    # 清理
    kill $monitor_pid 2>/dev/null || true
}

# 显示使用说明
show_usage() {
    echo -e "${BLUE}=== 使用说明 ===${NC}"
    cat << EOF

1. 基本使用流程:
   a) 确保 roscore 正在运行
   b) 启动覆盖监控: rosrun auto_nav coverage_monitor.py
   c) 启动清扫任务: roslaunch auto_nav sequential_clean.launch
   d) 观察CSV文件生成: ls -la /home/getting/tmp/sweeping_robot_realtime_data_*.csv

2. 手动命令:
   # 启动监控
   rosrun auto_nav coverage_monitor.py
   
   # 监控CSV生成
   watch -n 5 'ls -la /home/getting/tmp/sweeping_robot_realtime_data_*.csv'
   
   # 查看实时数据
   tail -f /home/getting/tmp/sweeping_robot_realtime_data_*.csv

3. 数据分析:
   # Python分析 (需要pandas)
   python3 analyze_csv_data.py
   
   # Excel分析
   用Excel打开CSV文件进行分析

4. 注意事项:
   - CSV文件每30秒保存一次
   - 文件保存在/home/getting/tmp目录，重启后会清空
   - 支持多个任务同时运行，文件名不冲突
   - 不影响原有的最终报告功能

EOF
}

# 主程序
main() {
    # 执行基本检查
    check_files
    check_ros_environment
    check_permissions
    
    # 显示功能信息
    show_feature_info
    show_csv_fields
    
    # 如果有命令行参数，直接执行对应功能
    if [ $# -gt 0 ]; then
        case "$1" in
            "monitor")
                start_coverage_monitor
                ;;
            "full")
                start_full_system "${2:-hospital_0.1}"
                ;;
            "watch")
                monitor_csv_files
                ;;
            "usage")
                show_usage
                ;;
            *)
                echo "未知参数: $1"
                echo "支持的参数: monitor, full, watch, usage"
                exit 1
                ;;
        esac
        return
    fi
    
    # 交互式菜单
    while true; do
        show_menu
        read -p "请选择操作 [0-6]: " choice
        
        case $choice in
            1)
                start_coverage_monitor
                ;;
            2)
                read -p "请输入地图名称 [hospital_0.1]: " map_name
                map_name=${map_name:-hospital_0.1}
                start_full_system "$map_name"
                ;;
            3)
                echo "启动演示系统..."
                cd "$PROJECT_ROOT"
                ./demo_system.sh
                ;;
            4)
                monitor_csv_files
                ;;
            5)
                if command -v python3 &> /dev/null && python3 -c "import pandas" &> /dev/null; then
                    python3 analyze_csv_data.py
                else
                    print_error "缺少Python pandas库，请安装: pip3 install pandas matplotlib"
                fi
                ;;
            6)
                show_usage
                read -p "按回车继续..."
                ;;
            0)
                echo "退出"
                exit 0
                ;;
            *)
                print_error "无效选择，请重新输入"
                ;;
        esac
        
        echo ""
        read -p "按回车继续..."
    done
}

# 捕获Ctrl+C
trap 'echo -e "\n${YELLOW}操作被用户中断${NC}"; exit 0' INT

# 执行主程序
main "$@"
