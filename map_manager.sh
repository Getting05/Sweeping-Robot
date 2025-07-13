#!/bin/bash

# 地图管理主脚本
# 集成地图切换、参数化等功能

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

show_menu() {
    clear
    echo "🗺️  扫地机器人地图管理系统"
    echo "================================"
    echo
    echo "1. 显示当前地图"
    echo "2. 列出所有可用地图"
    echo "3. 切换地图"
    echo "4. 添加新地图到配置"
    echo "5. 创建参数化launch文件"
    echo "6. 恢复原始launch文件"
    echo "7. 验证地图文件完整性"
    echo "8. 快速切换到hospital_0.1"
    echo "9. 快速切换到kitchen_5"
    echo "0. 退出"
    echo
}

# 添加新地图到配置
add_new_map() {
    echo "📋 添加新地图到配置"
    echo
    
    read -p "请输入新地图名称 (不包含扩展名): " map_name
    
    if [[ -z "$map_name" ]]; then
        echo "❌ 地图名称不能为空"
        return 1
    fi
    
    # 检查地图文件是否存在
    local map_dir="/home/getting/Sweeping-Robot/src/auto_nav/map"
    if [[ ! -f "${map_dir}/${map_name}.yaml" ]] || [[ ! -f "${map_dir}/${map_name}.pgm" ]]; then
        echo "⚠️  警告: 地图文件不存在"
        echo "   期望文件: ${map_dir}/${map_name}.yaml"
        echo "   期望文件: ${map_dir}/${map_name}.pgm"
        
        read -p "是否仍要添加到配置? (y/N): " confirm
        if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
            echo "取消添加"
            return 1
        fi
    fi
    
    # 检查是否已存在
    if grep -q "\"$map_name\"" "${SCRIPT_DIR}/map_config.sh"; then
        echo "ℹ️  地图 '$map_name' 已在配置中"
        return 0
    fi
    
    # 添加到配置文件
    sed -i "/# 在这里添加更多地图/i\\    \"$map_name\"" "${SCRIPT_DIR}/map_config.sh"
    
    echo "✅ 已添加地图 '$map_name' 到配置"
}

# 验证所有地图文件
verify_maps() {
    echo "🔍 验证地图文件完整性"
    echo "===================="
    
    source "${SCRIPT_DIR}/map_config.sh"
    
    local valid_count=0
    local total_count=${#AVAILABLE_MAPS[@]}
    
    for map in "${AVAILABLE_MAPS[@]}"; do
        echo -n "检查 $map ... "
        if validate_map "$map" 2>/dev/null; then
            echo "✅ 完整"
            ((valid_count++))
        else
            echo "❌ 缺失文件"
        fi
    done
    
    echo
    echo "📊 验证结果: $valid_count/$total_count 个地图完整"
    
    if [[ $valid_count -lt $total_count ]]; then
        echo
        echo "💡 提示: 缺失的地图文件需要:"
        echo "  1. .yaml 文件 (地图元数据)"
        echo "  2. .pgm 文件 (地图图像)"
        echo "  放置在: /home/getting/Sweeping-Robot/src/auto_nav/map/"
    fi
}

# 交互式地图切换
interactive_map_switch() {
    source "${SCRIPT_DIR}/map_config.sh"
    
    echo "🔄 选择要切换的地图"
    echo "=================="
    echo
    
    local index=1
    local map_array=()
    
    for map in "${AVAILABLE_MAPS[@]}"; do
        if [[ "$map" == "$CURRENT_MAP" ]]; then
            marker="🟢 (当前)"
        else
            marker="⚪"
        fi
        
        echo "$index. $marker $map"
        map_array+=("$map")
        ((index++))
    done
    
    echo
    read -p "请选择地图编号 (1-${#AVAILABLE_MAPS[@]}): " choice
    
    if [[ "$choice" =~ ^[0-9]+$ ]] && [[ $choice -ge 1 ]] && [[ $choice -le ${#AVAILABLE_MAPS[@]} ]]; then
        local selected_map="${map_array[$((choice-1))]}"
        echo
        echo "您选择了: $selected_map"
        read -p "确认切换? (y/N): " confirm
        
        if [[ "$confirm" == "y" || "$confirm" == "Y" ]]; then
            "${SCRIPT_DIR}/switch_map.sh" "$selected_map"
        else
            echo "取消切换"
        fi
    else
        echo "❌ 无效选择"
    fi
}

# 主循环
main() {
    cd "$SCRIPT_DIR"
    
    while true; do
        show_menu
        read -p "请选择操作 (0-9): " choice
        
        case $choice in
            1)
                echo
                ./switch_map.sh --current
                read -p "按回车继续..."
                ;;
            2)
                echo
                ./switch_map.sh --list
                read -p "按回车继续..."
                ;;
            3)
                echo
                interactive_map_switch
                read -p "按回车继续..."
                ;;
            4)
                echo
                add_new_map
                read -p "按回车继续..."
                ;;
            5)
                echo
                ./parameterize_launch.sh --parameterize
                read -p "按回车继续..."
                ;;
            6)
                echo
                ./parameterize_launch.sh --restore
                read -p "按回车继续..."
                ;;
            7)
                echo
                verify_maps
                read -p "按回车继续..."
                ;;
            8)
                echo
                echo "🏥 快速切换到 hospital_0.1"
                ./switch_map.sh hospital_0.1
                read -p "按回车继续..."
                ;;
            9)
                echo
                echo "🍴 快速切换到 kitchen_5"
                ./switch_map.sh kitchen_5
                read -p "按回车继续..."
                ;;
            0)
                echo "👋 再见！"
                exit 0
                ;;
            *)
                echo "❌ 无效选择，请重试"
                sleep 2
                ;;
        esac
    done
}

# 检查依赖脚本
check_dependencies() {
    local missing=0
    
    if [[ ! -f "${SCRIPT_DIR}/map_config.sh" ]]; then
        echo "❌ 缺少文件: map_config.sh"
        missing=1
    fi
    
    if [[ ! -f "${SCRIPT_DIR}/switch_map.sh" ]]; then
        echo "❌ 缺少文件: switch_map.sh"
        missing=1
    fi
    
    if [[ ! -f "${SCRIPT_DIR}/parameterize_launch.sh" ]]; then
        echo "❌ 缺少文件: parameterize_launch.sh"
        missing=1
    fi
    
    if [[ $missing -eq 1 ]]; then
        echo "请确保所有必要文件都在当前目录中"
        exit 1
    fi
    
    # 设置执行权限
    chmod +x "${SCRIPT_DIR}/switch_map.sh"
    chmod +x "${SCRIPT_DIR}/parameterize_launch.sh"
}

# 启动
check_dependencies
main
