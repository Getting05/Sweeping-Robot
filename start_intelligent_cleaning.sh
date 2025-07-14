#!/bin/bash

# 扫地机器人智能清扫系统 - 一键启动脚本 v2.2
# 集成覆盖率监控、CSV数据记录、自动重启等所有功能

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# 项目配置
PROJECT_ROOT="/home/getting/Sweeping-Robot"
DEFAULT_MAP="hospital_0.1"

# 函数定义
print_banner() {
    echo -e "${PURPLE}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║          扫地机器人智能清扫系统 v2.2                        ║"
    echo "║                                                              ║"
    echo "║  ✨ 实时覆盖率监控     📊 CSV数据记录                       ║"
    echo "║  🔄 智能自动重启       🗺️  地图管理                         ║"
    echo "║  📈 性能评估报告       🚀 一键启动                          ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

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

# 显示主菜单
show_main_menu() {
    echo -e "${BLUE}=== 主功能菜单 ===${NC}"
    echo "1) 🚀 智能启动 (自动重启管理器 + 完整清扫系统)"
    echo "2) 📊 仅启动覆盖监控 (包含CSV记录)"
    echo "3) 🔄 测试自动重启功能"
    echo "4) 🗺️  地图管理"
    echo "5) 📈 数据分析工具"
    echo "6) 🔧 系统管理"
    echo "7) 📖 使用说明"
    echo "0) 退出"
    echo ""
}

# 显示系统管理菜单
show_system_menu() {
    echo -e "${BLUE}=== 系统管理 ===${NC}"
    echo "1) 检查系统状态"
    echo "2) 停止所有服务"
    echo "3) 清理临时文件"
    echo "4) 查看运行日志"
    echo "5) 编译项目"
    echo "6) 环境检查"
    echo "0) 返回主菜单"
    echo ""
}

# 显示数据分析菜单
show_analysis_menu() {
    echo -e "${BLUE}=== 数据分析工具 ===${NC}"
    echo "1) 分析最新CSV数据"
    echo "2) 监控实时CSV生成"
    echo "3) 查看重启报告"
    echo "4) 生成性能图表"
    echo "5) 导出分析报告"
    echo "0) 返回主菜单"
    echo ""
}

# 检查环境
check_environment() {
    print_info "检查系统环境..."
    
    # 检查ROS环境
    if [ -z "$ROS_DISTRO" ]; then
        print_warning "ROS环境未设置，尝试source"
        cd "$PROJECT_ROOT"
        if [ -f "devel/setup.bash" ]; then
            source devel/setup.bash
            print_status "已source ROS环境"
        else
            print_error "ROS环境配置失败"
            return 1
        fi
    else
        print_status "ROS环境: $ROS_DISTRO"
    fi
    
    # 检查项目文件
    local required_files=(
        "auto_restart_manager.sh"
        "src/auto_nav/scripts/coverage_monitor.py"
        "src/auto_nav/launch/sequential_clean.launch"
    )
    
    for file in "${required_files[@]}"; do
        if [ -f "$PROJECT_ROOT/$file" ]; then
            print_status "找到: $file"
        else
            print_error "缺失: $file"
            return 1
        fi
    done
    
    return 0
}

# 智能启动 (推荐)
smart_start() {
    print_info "=== 智能启动模式 ==="
    
    # 获取地图参数
    read -p "请输入地图名称 [$DEFAULT_MAP]: " map_name
    map_name=${map_name:-$DEFAULT_MAP}
    
    print_info "启动配置:"
    print_info "- 地图: $map_name"
    print_info "- 覆盖率监控: 启用"
    print_info "- CSV数据记录: 启用 (每30秒)"
    print_info "- 自动重启: 启用 (120秒停滞阈值)"
    print_info "- 最大重启次数: 500次"
    
    echo ""
    read -p "确认启动? [Y/n]: " confirm
    if [[ "$confirm" =~ ^[Nn]$ ]]; then
        print_info "取消启动"
        return
    fi
    
    # 修改自动重启管理器的启动命令以包含地图参数
    local temp_manager="/tmp/auto_restart_manager_temp.sh"
    sed "s|sequential_clean.launch|sequential_clean.launch map_name:=$map_name|g" \
        "$PROJECT_ROOT/auto_restart_manager.sh" > "$temp_manager"
    chmod +x "$temp_manager"
    
    print_info "启动智能管理器..."
    "$temp_manager" start
    
    # 清理临时文件
    rm -f "$temp_manager"
}

# 仅启动覆盖监控
start_coverage_only() {
    print_info "=== 仅启动覆盖监控 ==="
    
    # 检查roscore
    if ! pgrep -f roscore > /dev/null; then
        print_info "启动roscore..."
        roscore &
        sleep 3
    fi
    
    cd "$PROJECT_ROOT"
    source devel/setup.bash
    
    print_info "启动覆盖率监控 (包含CSV记录和自动重启功能)..."
    print_info "CSV文件位置: /tmp/sweeping_robot_realtime_data_*.csv"
    print_info "按 Ctrl+C 停止监控"
    
    rosrun auto_nav coverage_monitor.py
}

# 测试自动重启功能
test_auto_restart() {
    print_info "=== 测试自动重启功能 ==="
    
    python3 "$PROJECT_ROOT/test_auto_restart.py"
    
    echo ""
    read -p "是否进行实际重启测试? [y/N]: " test_confirm
    if [[ "$test_confirm" =~ ^[Yy]$ ]]; then
        print_warning "将启动测试模式，覆盖率阈值设为30秒以便快速测试"
        
        # 创建测试配置的临时监控脚本
        local temp_monitor="/tmp/coverage_monitor_test.py"
        sed 's/coverage_stagnation_threshold = 120.0/coverage_stagnation_threshold = 30.0/g' \
            "$PROJECT_ROOT/src/auto_nav/scripts/coverage_monitor.py" > "$temp_monitor"
        
        print_info "启动测试模式 (30秒停滞阈值)..."
        chmod +x "$temp_monitor"
        python3 "$temp_monitor"
        
        rm -f "$temp_monitor"
    fi
}

# 地图管理
map_management() {
    print_info "=== 地图管理 ==="
    
    if [ -f "$PROJECT_ROOT/map_manager.sh" ]; then
        "$PROJECT_ROOT/map_manager.sh"
    else
        print_error "地图管理器未找到"
    fi
}

# 数据分析工具
data_analysis() {
    while true; do
        show_analysis_menu
        read -p "请选择操作 [0-5]: " choice
        
        case $choice in
            1)
                if command -v python3 &> /dev/null && python3 -c "import pandas" &> /dev/null 2>&1; then
                    python3 "$PROJECT_ROOT/analyze_csv_data.py"
                else
                    print_error "缺少Python pandas库"
                    print_info "安装命令: pip3 install pandas matplotlib"
                fi
                ;;
            2)
                "$PROJECT_ROOT/test_csv_monitor.sh" watch
                ;;
            3)
                print_info "重启报告列表:"
                ls -la /tmp/auto_restart_report_*.txt 2>/dev/null || print_warning "无重启报告"
                
                reports=($(ls /tmp/auto_restart_report_*.txt 2>/dev/null || true))
                if [ ${#reports[@]} -gt 0 ]; then
                    echo ""
                    read -p "查看最新报告? [Y/n]: " view_report
                    if [[ ! "$view_report" =~ ^[Nn]$ ]]; then
                        cat "${reports[-1]}"
                    fi
                fi
                ;;
            4)
                print_info "启动图表生成..."
                if command -v python3 &> /dev/null; then
                    python3 -c "
import matplotlib.pyplot as plt
import numpy as np
print('生成示例性能图表...')
plt.figure(figsize=(10, 6))
plt.plot([1,2,3,4,5], [20,40,60,80,90], 'b-', label='覆盖率(%)')
plt.xlabel('时间(分钟)')
plt.ylabel('覆盖率(%)')
plt.title('扫地机器人覆盖率变化')
plt.legend()
plt.grid(True)
plt.savefig('/tmp/performance_chart.png', dpi=150)
print('图表已保存: /tmp/performance_chart.png')
plt.show()
"
                else
                    print_error "Python3未安装"
                fi
                ;;
            5)
                print_info "生成分析报告..."
                local report_file="/tmp/system_analysis_$(date +%Y%m%d_%H%M%S).txt"
                cat > "$report_file" << EOF
=== 扫地机器人系统分析报告 ===
生成时间: $(date)
系统版本: v2.2

=== 功能状态 ===
✓ 实时覆盖率监控
✓ CSV数据记录 (每30秒)
✓ 智能自动重启 (20秒阈值)
✓ 地图管理系统
✓ 性能评估报告

=== 文件统计 ===
CSV文件数: $(ls /tmp/sweeping_robot_realtime_data_*.csv 2>/dev/null | wc -l)
重启报告数: $(ls /tmp/auto_restart_report_*.txt 2>/dev/null | wc -l)
日志文件数: $(ls /tmp/auto_restart_logs/*.log 2>/dev/null | wc -l || echo 0)

=== 系统状态 ===
ROS环境: ${ROS_DISTRO:-未设置}
roscore状态: $(pgrep -f roscore > /dev/null && echo "运行中" || echo "未运行")
管理器状态: $(pgrep -f auto_restart_manager > /dev/null && echo "运行中" || echo "未运行")

=== 推荐操作 ===
1. 定期清理临时文件
2. 备份重要的CSV数据
3. 监控重启频率
4. 优化系统参数
=====================================
EOF
                print_status "报告已保存: $report_file"
                cat "$report_file"
                ;;
            0)
                return
                ;;
            *)
                print_error "无效选择"
                ;;
        esac
        
        echo ""
        read -p "按回车继续..."
    done
}

# 系统管理
system_management() {
    while true; do
        show_system_menu
        read -p "请选择操作 [0-6]: " choice
        
        case $choice in
            1)
                print_info "=== 系统状态检查 ==="
                "$PROJECT_ROOT/auto_restart_manager.sh" status 2>/dev/null || print_warning "管理器未运行"
                
                echo ""
                print_info "ROS节点状态:"
                rosnode list 2>/dev/null | grep -E "(coverage_monitor|sequential_goal|path_planning)" || print_warning "相关节点未运行"
                
                echo ""
                print_info "进程状态:"
                ps aux | grep -E "(roscore|roslaunch|coverage_monitor)" | grep -v grep || print_warning "无相关进程"
                ;;
            2)
                print_warning "停止所有服务..."
                "$PROJECT_ROOT/auto_restart_manager.sh" stop 2>/dev/null || true
                pkill -f "sequential_clean" || true
                pkill -f "coverage_monitor" || true
                print_status "所有服务已停止"
                ;;
            3)
                print_info "清理临时文件..."
                rm -f /tmp/sweeping_robot_realtime_data_*.csv
                rm -f /tmp/auto_restart_report_*.txt
                rm -rf /tmp/auto_restart_logs/
                print_status "临时文件已清理"
                ;;
            4)
                print_info "=== 查看运行日志 ==="
                echo "1) ROS节点日志"
                echo "2) 管理器日志"
                echo "3) 系统日志"
                read -p "选择日志类型 [1-3]: " log_choice
                
                case $log_choice in
                    1)
                        rosnode logs coverage_monitor 2>/dev/null || print_warning "无coverage_monitor日志"
                        ;;
                    2)
                        if [ -d "/tmp/auto_restart_logs" ]; then
                            ls -la /tmp/auto_restart_logs/
                            latest_log=$(find /tmp/auto_restart_logs -name "*.log" -type f -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)
                            if [ -n "$latest_log" ]; then
                                print_info "最新日志: $latest_log"
                                tail -20 "$latest_log"
                            fi
                        else
                            print_warning "无管理器日志"
                        fi
                        ;;
                    3)
                        journalctl --no-pager -u roscore -n 20 2>/dev/null || print_warning "无系统日志"
                        ;;
                esac
                ;;
            5)
                print_info "编译项目..."
                cd "$PROJECT_ROOT"
                catkin_make
                print_status "编译完成"
                ;;
            6)
                check_environment
                ;;
            0)
                return
                ;;
            *)
                print_error "无效选择"
                ;;
        esac
        
        echo ""
        read -p "按回车继续..."
    done
}

# 显示使用说明
show_usage() {
    print_info "=== 使用说明 ==="
    cat << EOF

🚀 快速开始:
1. 选择 "智能启动" 获得最佳体验
2. 系统会自动启动覆盖监控、CSV记录、自动重启等所有功能
3. 当覆盖率20秒无变化时会自动重启清扫任务

📊 功能特性:
• 实时覆盖率监控和可视化
• 每30秒自动保存CSV数据
• 覆盖率停滞2分钟自动重启
• 智能地图管理和切换
• 完整的性能评估报告

🔧 文件位置:
• CSV数据: /tmp/sweeping_robot_realtime_data_*.csv
• 重启报告: /tmp/auto_restart_report_*.txt
• 运行日志: /tmp/auto_restart_logs/

📖 详细文档:
• AUTO_RESTART_GUIDE.md - 自动重启功能指南
• CSV_FEATURE_GUIDE.md - CSV功能使用指南
• PROJECT_STATUS.md - 完整项目状态

⚡ 快捷命令:
• 智能启动: ./start_intelligent_cleaning.sh
• 状态检查: ./auto_restart_manager.sh status
• 停止服务: ./auto_restart_manager.sh stop
• 数据分析: python3 analyze_csv_data.py

EOF
}

# 主函数
main() {
    print_banner
    
    # 检查环境
    if ! check_environment; then
        print_error "环境检查失败，请修复后重试"
        exit 1
    fi
    
    while true; do
        show_main_menu
        read -p "请选择操作 [0-7]: " choice
        
        case $choice in
            1)
                smart_start
                ;;
            2)
                start_coverage_only
                ;;
            3)
                test_auto_restart
                ;;
            4)
                map_management
                ;;
            5)
                data_analysis
                ;;
            6)
                system_management
                ;;
            7)
                show_usage
                read -p "按回车继续..."
                ;;
            0)
                print_info "感谢使用扫地机器人智能清扫系统！"
                exit 0
                ;;
            *)
                print_error "无效选择，请重新输入"
                ;;
        esac
        
        echo ""
    done
}

# 命令行参数处理
case "${1:-interactive}" in
    "smart"|"start")
        check_environment && smart_start
        ;;
    "coverage")
        check_environment && start_coverage_only
        ;;
    "test")
        check_environment && test_auto_restart
        ;;
    "map")
        check_environment && map_management
        ;;
    "status")
        check_environment && "$PROJECT_ROOT/auto_restart_manager.sh" status
        ;;
    "stop")
        "$PROJECT_ROOT/auto_restart_manager.sh" stop
        ;;
    "help")
        show_usage
        ;;
    "interactive")
        main
        ;;
    *)
        echo "用法: $0 {smart|coverage|test|map|status|stop|help|interactive}"
        echo "  smart       - 智能启动 (推荐)"
        echo "  coverage    - 仅启动覆盖监控"
        echo "  test        - 测试自动重启功能"
        echo "  map         - 地图管理"
        echo "  status      - 检查系统状态"
        echo "  stop        - 停止所有服务"
        echo "  help        - 显示使用说明"
        echo "  interactive - 交互式菜单 (默认)"
        exit 1
        ;;
esac
