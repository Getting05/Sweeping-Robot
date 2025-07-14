#!/bin/bash

# 自动重启清扫系统管理脚本
# 监控覆盖率并在停滞时自动重启清扫任务

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 配置参数
PROJECT_ROOT="/home/getting/Sweeping-Robot"
LAUNCH_COMMAND="roslaunch auto_nav sequential_clean.launch"
LOG_DIR="/home/getting/tmp/auto_restart_logs"
MAX_RESTART_COUNT=500  # 最大重启次数
RESTART_DELAY=60      # 重启间隔秒数

# 全局变量
restart_count=0
start_time=$(date +%s)

# 创建日志目录
mkdir -p "$LOG_DIR"

# 函数定义
print_status() {
    echo -e "${GREEN}[$(date '+%Y-%m-%d %H:%M:%S')] ✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[$(date '+%Y-%m-%d %H:%M:%S')] ⚠${NC} $1"
}

print_error() {
    echo -e "${RED}[$(date '+%Y-%m-%d %H:%M:%S')] ✗${NC} $1"
}

print_info() {
    echo -e "${BLUE}[$(date '+%Y-%m-%d %H:%M:%S')] ℹ${NC} $1"
}

# 检查ROS环境
check_ros_environment() {
    print_info "检查ROS环境..."
    
    if [ -z "$ROS_DISTRO" ]; then
        print_warning "ROS环境变量未设置，source setup文件"
        cd "$PROJECT_ROOT"
        source devel/setup.bash
    fi
    
    if ! pgrep -f roscore > /dev/null; then
        print_info "启动roscore..."
        roscore &
        sleep 3
    fi
    
    print_status "ROS环境就绪"
}

# 清理之前的进程
cleanup_previous_processes() {
    print_info "清理之前的进程..."
    
    # 关闭之前的launch进程
    pkill -f "sequential_clean.launch" || true
    pkill -f "coverage_monitor" || true
    pkill -f "sequential_goal" || true
    pkill -f "path_planning" || true
    
    # 等待进程完全关闭
    sleep 60
    
    print_status "进程清理完成"
}

# 启动清扫系统
start_cleaning_system() {
    local attempt=$1
    
    print_info "启动清扫系统 (第 $attempt 次尝试)..."
    
    cd "$PROJECT_ROOT"
    source devel/setup.bash
    
    # 创建本次启动的日志文件
    local log_file="$LOG_DIR/restart_${attempt}_$(date +%Y%m%d_%H%M%S).log"
    
    print_info "日志文件: $log_file"
    print_info "执行命令: $LAUNCH_COMMAND"
    
    # 在后台启动清扫系统，并记录日志
    nohup bash -c "
        cd '$PROJECT_ROOT'
        source devel/setup.bash
        echo '=== 清扫系统启动日志 ===' >> '$log_file'
        echo '启动时间: $(date)' >> '$log_file'
        echo '重启次数: $attempt' >> '$log_file'
        echo '命令: $LAUNCH_COMMAND' >> '$log_file'
        echo '========================' >> '$log_file'
        
        $LAUNCH_COMMAND >> '$log_file' 2>&1
    " &
    
    local launch_pid=$!
    echo $launch_pid > "$LOG_DIR/current_launch.pid"
    
    print_status "清扫系统已启动 (PID: $launch_pid)"
    print_info "监控日志: tail -f $log_file"
    
    return 0
}

# 监控系统状态
monitor_system() {
    local check_interval=30  # 每30秒检查一次
    local last_restart_check=$(date +%s)
    
    print_info "开始监控系统状态 (检查间隔: ${check_interval}s)..."
    
    while true; do
        sleep $check_interval
        
        current_time=$(date +%s)
        running_time=$((current_time - start_time))
        
        # 检查是否有自动重启报告生成
        restart_reports=($(find /home/getting/tmp -name "auto_restart_report_*.txt" -newer /home/getting/tmp/monitor_start_time 2>/dev/null || true))
        
        if [ ${#restart_reports[@]} -gt 0 ]; then
            # 发现有重启报告，说明coverage_monitor触发了重启
            local latest_report="${restart_reports[-1]}"
            print_warning "检测到自动重启触发信号: $latest_report"
            
            # 读取并显示重启报告
            if [ -f "$latest_report" ]; then
                print_info "重启报告内容:"
                cat "$latest_report"
            fi
            
            # 执行重启
            handle_auto_restart
            
            # 清理重启报告标记
            touch /home/getting/tmp/monitor_start_time
        fi
        
        # 检查清扫进程是否还在运行
        if [ -f "$LOG_DIR/current_launch.pid" ]; then
            local pid=$(cat "$LOG_DIR/current_launch.pid")
            if ! kill -0 $pid 2>/dev/null; then
                print_error "清扫进程 (PID: $pid) 已退出"
                handle_process_exit
            fi
        fi
        
        # 定期状态报告
        if [ $((current_time - last_restart_check)) -ge 300 ]; then  # 每5分钟报告一次
            print_info "系统运行状态: 运行时间 ${running_time}s, 重启次数 $restart_count"
            last_restart_check=$current_time
        fi
    done
}

# 处理自动重启
handle_auto_restart() {
    restart_count=$((restart_count + 1))
    
    print_warning "处理自动重启 (第 $restart_count 次)..."
    
    # 检查是否超过最大重启次数
    if [ $restart_count -gt $MAX_RESTART_COUNT ]; then
        print_error "已达到最大重启次数 ($MAX_RESTART_COUNT)，停止自动重启"
        print_error "请检查系统配置或手动干预"
        exit 1
    fi
    
    # 清理当前进程
    cleanup_previous_processes
    
    # 等待一段时间再重启
    print_info "等待 ${RESTART_DELAY} 秒后重启..."
    sleep $RESTART_DELAY
    
    # 启动新的清扫系统
    start_cleaning_system $restart_count
    
    print_status "自动重启完成"
}

# 处理进程意外退出
handle_process_exit() {
    print_warning "清扫进程意外退出，准备重启..."
    handle_auto_restart
}

# 信号处理
cleanup_and_exit() {
    print_info "接收到退出信号，清理资源..."
    
    # 清理进程
    cleanup_previous_processes
    
    # 生成最终报告
    local final_report="$LOG_DIR/final_report_$(date +%Y%m%d_%H%M%S).txt"
    cat > "$final_report" << EOF
=== 自动重启管理器最终报告 ===
结束时间: $(date)
总运行时间: $(($(date +%s) - start_time)) 秒
总重启次数: $restart_count
日志目录: $LOG_DIR

重启历史:
$(find "$LOG_DIR" -name "restart_*.log" -exec basename {} \; | sort)

自动重启报告:
$(find /home/getting/tmp -name "auto_restart_report_*.txt" -exec basename {} \; | sort)
============================
EOF
    
    print_info "最终报告已保存: $final_report"
    print_info "自动重启管理器退出"
    exit 0
}

# 主函数
main() {
    print_info "=== 自动重启清扫系统管理器启动 ==="
    print_info "项目路径: $PROJECT_ROOT"
    print_info "启动命令: $LAUNCH_COMMAND"
    print_info "最大重启次数: $MAX_RESTART_COUNT"
    print_info "重启间隔: ${RESTART_DELAY}s"
    
    # 创建监控起始时间标记
    touch /home/getting/tmp/monitor_start_time
    
    # 设置信号处理
    trap cleanup_and_exit SIGTERM SIGINT
    
    # 检查环境
    check_ros_environment
    
    # 清理之前的进程
    cleanup_previous_processes
    
    # 启动初始清扫系统
    start_cleaning_system 1
    restart_count=1
    
    # 开始监控
    monitor_system
}

# 检查命令行参数
case "${1:-start}" in
    "start")
        main
        ;;
    "stop")
        print_info "停止自动重启管理器..."
        pkill -f "auto_restart_manager.sh" || true
        cleanup_previous_processes
        print_status "已停止"
        ;;
    "status")
        print_info "检查自动重启管理器状态..."
        if pgrep -f "auto_restart_manager.sh" > /dev/null; then
            print_status "自动重启管理器正在运行"
            if [ -f "$LOG_DIR/current_launch.pid" ]; then
                local pid=$(cat "$LOG_DIR/current_launch.pid")
                if kill -0 $pid 2>/dev/null; then
                    print_status "清扫系统正在运行 (PID: $pid)"
                else
                    print_warning "清扫系统未运行"
                fi
            fi
        else
            print_warning "自动重启管理器未运行"
        fi
        ;;
    "logs")
        print_info "显示最近的日志..."
        if [ -d "$LOG_DIR" ]; then
            ls -la "$LOG_DIR"
            echo ""
            print_info "最新日志内容:"
            latest_log=$(find "$LOG_DIR" -name "restart_*.log" -type f -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)
            if [ -n "$latest_log" ]; then
                tail -20 "$latest_log"
            fi
        else
            print_warning "日志目录不存在"
        fi
        ;;
    *)
        echo "用法: $0 {start|stop|status|logs}"
        echo "  start  - 启动自动重启管理器"
        echo "  stop   - 停止自动重启管理器"
        echo "  status - 检查运行状态"
        echo "  logs   - 显示日志"
        exit 1
        ;;
esac
