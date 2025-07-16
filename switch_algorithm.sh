#!/bin/bash

# 简易算法切换脚本
# 通过修改配置文件来切换路径规划算法

CONFIG_FILE="/home/getting/Sweeping-Robot/src/auto_nav/config/path_planning_algorithms.yaml"

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# 显示当前算法
show_current_algorithm() {
    if [ -f "$CONFIG_FILE" ]; then
        current_algo=$(grep "algorithm_type:" "$CONFIG_FILE" | sed 's/algorithm_type: *"//' | sed 's/"//')
        current_weight=$(grep "heuristic_weight:" "$CONFIG_FILE" | sed 's/heuristic_weight: *//')
        current_pattern=$(grep "^coverage_pattern:" "$CONFIG_FILE" | sed 's/coverage_pattern: *//')
        
        echo -e "${BLUE}=== 当前算法配置 ===${NC}"
        echo "算法类型: $current_algo"
        echo "A*启发式权重: $current_weight"
        echo "MCP覆盖模式: $current_pattern"
        echo ""
    else
        print_warning "配置文件不存在: $CONFIG_FILE"
    fi
}

# 切换算法类型
switch_algorithm() {
    local new_algo="$1"
    print_info "切换算法到: $new_algo"
    
    # 备份原配置
    cp "$CONFIG_FILE" "${CONFIG_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    
    # 修改算法类型
    sed -i "s/algorithm_type: *\".*\"/algorithm_type: \"$new_algo\"/" "$CONFIG_FILE"
    
    print_success "算法已切换到: $new_algo"
}

# 设置A*参数
set_astar_params() {
    local weight="$1"
    print_info "设置A*启发式权重为: $weight"
    
    sed -i "s/heuristic_weight: .*/heuristic_weight: $weight/" "$CONFIG_FILE"
    
    print_success "A*参数已更新"
}

# 设置MCP参数
set_mcp_params() {
    local pattern="$1"
    print_info "设置MCP覆盖模式为: $pattern"
    
    sed -i "s/coverage_pattern: .*/coverage_pattern: $pattern/" "$CONFIG_FILE"
    
    case $pattern in
        0) print_success "MCP模式已设置为: 蛇形覆盖" ;;
        1) print_success "MCP模式已设置为: 螺旋覆盖" ;;
        2) print_success "MCP模式已设置为: 分区覆盖" ;;
        *) print_warning "未知的覆盖模式: $pattern" ;;
    esac
}

# 显示菜单
show_menu() {
    echo -e "${BLUE}=== 算法切换工具 ===${NC}"
    echo "1) 查看当前算法"
    echo "2) 切换到神经网络算法 (Neural Network)"
    echo "3) 切换到A*算法"
    echo "4) 切换到D*算法"
    echo "5) 切换到MCP算法"
    echo "6) 设置A*启发式权重"
    echo "7) 设置MCP覆盖模式"
    echo "8) 显示配置文件内容"
    echo "0) 退出"
    echo ""
}

# 主函数
main() {
    while true; do
        show_menu
        read -p "请选择操作 [0-8]: " choice
        
        case $choice in
            1)
                show_current_algorithm
                ;;
            2)
                switch_algorithm "neural_network"
                ;;
            3)
                switch_algorithm "astar"
                ;;
            4)
                switch_algorithm "dstar"
                ;;
            5)
                switch_algorithm "mcp"
                ;;
            6)
                read -p "请输入A*启发式权重 (默认1.0): " weight
                weight=${weight:-1.0}
                set_astar_params "$weight"
                ;;
            7)
                echo "覆盖模式选项:"
                echo "  0 - 蛇形覆盖"
                echo "  1 - 螺旋覆盖"
                echo "  2 - 分区覆盖"
                read -p "请输入覆盖模式 [0-2]: " pattern
                if [[ "$pattern" =~ ^[0-2]$ ]]; then
                    set_mcp_params "$pattern"
                else
                    print_warning "无效的覆盖模式，请输入0-2"
                fi
                ;;
            8)
                echo -e "${BLUE}=== 配置文件内容 ===${NC}"
                cat "$CONFIG_FILE"
                echo ""
                ;;
            0)
                print_info "退出算法切换工具"
                break
                ;;
            *)
                print_warning "无效选择，请重新输入"
                ;;
        esac
        
        read -p "按回车键继续..."
        echo ""
    done
}

# 命令行参数处理
if [ $# -eq 0 ]; then
    # 交互模式
    main
else
    # 命令行模式
    case "$1" in
        "show"|"current")
            show_current_algorithm
            ;;
        "neural"|"neural_network")
            switch_algorithm "neural_network"
            ;;
        "astar"|"a*")
            switch_algorithm "astar"
            if [ -n "$2" ]; then
                set_astar_params "$2"
            fi
            ;;
        "dstar"|"d*")
            switch_algorithm "dstar"
            ;;
        "mcp")
            switch_algorithm "mcp"
            if [ -n "$2" ]; then
                set_mcp_params "$2"
            fi
            ;;
        *)
            echo "用法: $0 [show|neural|astar|dstar|mcp] [参数]"
            echo "示例:"
            echo "  $0 show                    # 显示当前算法"
            echo "  $0 astar 1.5              # 切换到A*算法并设置权重"
            echo "  $0 mcp 1                  # 切换到MCP算法并设置螺旋模式"
            echo "  $0                        # 交互模式"
            ;;
    esac
fi
