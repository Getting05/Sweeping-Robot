#!/bin/bash

# 地图切换脚本
# 用于批量替换所有launch文件中的地图名称

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/map_config.sh"

# 需要修改的launch文件列表
LAUNCH_FILES=(
    "src/auto_nav/launch/sequential_clean.launch"
    "src/auto_nav/launch/clean_work_sequential.launch"
    "src/auto_nav/launch/sequential_clean_with_coverage.launch"
    "src/auto_nav/launch/sequential_clean_with_monitor.launch"
    "src/auto_nav/launch/clean_work_headless.launch"
    "src/auto_nav/launch/clean_work.launch"
    "src/auto_nav/launch/sequential_clean_with_coverage_viz.launch"
    "src/auto_nav/launch/sequential_clean_headless.launch"
)

# 显示使用说明
show_usage() {
    echo "地图切换工具"
    echo "用法: $0 [选项] [地图名称]"
    echo
    echo "选项:"
    echo "  -l, --list      显示所有可用地图"
    echo "  -c, --current   显示当前使用的地图"
    echo "  -h, --help      显示此帮助信息"
    echo
    echo "示例:"
    echo "  $0 hospital_0.1              # 切换到hospital_0.1地图"
    echo "  $0 -l                        # 列出所有可用地图"
    echo "  $0 -c                        # 显示当前地图"
    echo
    echo "可用地图:"
    for map in "${AVAILABLE_MAPS[@]}"; do
        echo "  - $map"
    done
}

# 显示当前地图
show_current_map() {
    echo "当前使用的地图: $CURRENT_MAP"
    
    # 验证当前地图文件是否存在
    if validate_map "$CURRENT_MAP"; then
        echo "✅ 地图文件完整"
    else
        echo "❌ 地图文件缺失"
    fi
}

# 列出所有可用地图
list_maps() {
    echo "可用地图列表:"
    echo "============="
    
    for map in "${AVAILABLE_MAPS[@]}"; do
        if [[ "$map" == "$CURRENT_MAP" ]]; then
            marker="🟢 (当前)"
        else
            marker="⚪"
        fi
        
        if validate_map "$map" 2>/dev/null; then
            status="✅"
        else
            status="❌"
        fi
        
        echo "$marker $map $status"
    done
    
    echo
    echo "说明:"
    echo "🟢 = 当前使用的地图"
    echo "✅ = 地图文件完整"
    echo "❌ = 地图文件缺失"
}

# 备份launch文件
backup_launch_files() {
    local timestamp=$(date +"%Y%m%d_%H%M%S")
    local backup_dir="${SCRIPT_DIR}/launch_backup_${timestamp}"
    
    echo "备份launch文件到: $backup_dir"
    mkdir -p "$backup_dir"
    
    for file in "${LAUNCH_FILES[@]}"; do
        if [[ -f "$file" ]]; then
            cp "$file" "$backup_dir/"
            echo "  备份: $file"
        fi
    done
    
    echo "✅ 备份完成"
}

# 切换地图
switch_map() {
    local new_map=$1
    local old_map=$CURRENT_MAP
    
    # 验证新地图
    if ! validate_map "$new_map"; then
        echo "❌ 无法切换到地图: $new_map"
        return 1
    fi
    
    echo "🔄 正在切换地图: $old_map -> $new_map"
    
    # 备份现有文件
    backup_launch_files
    
    # 替换所有launch文件中的地图名称
    local success_count=0
    local total_count=${#LAUNCH_FILES[@]}
    
    for file in "${LAUNCH_FILES[@]}"; do
        if [[ -f "$file" ]]; then
            # 使用sed替换地图名称
            sed -i "s|${old_map}\.yaml|${new_map}.yaml|g" "$file"
            
            # 验证替换是否成功
            if grep -q "${new_map}.yaml" "$file"; then
                echo "  ✅ 更新: $file"
                ((success_count++))
            else
                echo "  ❌ 失败: $file"
            fi
        else
            echo "  ⚠️  文件不存在: $file"
        fi
    done
    
    # 更新配置文件
    sed -i "s/CURRENT_MAP=.*/CURRENT_MAP=${new_map}/" "${SCRIPT_DIR}/map_config.sh"
    
    echo
    echo "📊 切换结果:"
    echo "  成功更新: $success_count/$total_count 个文件"
    echo "  新地图: $new_map"
    
    if [[ $success_count -eq $total_count ]]; then
        echo "🎉 地图切换完成！"
        echo
        echo "💡 建议:"
        echo "  1. 重新编译项目: catkin_make"
        echo "  2. 重启ROS系统测试新地图"
    else
        echo "⚠️  部分文件更新失败，请检查并手动修复"
    fi
}

# 恢复备份
restore_backup() {
    echo "可用的备份:"
    ls -la launch_backup_* 2>/dev/null || {
        echo "没有找到备份文件"
        return 1
    }
    
    echo
    read -p "请输入要恢复的备份目录名: " backup_dir
    
    if [[ -d "$backup_dir" ]]; then
        echo "🔄 正在恢复备份..."
        for file in "${LAUNCH_FILES[@]}"; do
            local filename=$(basename "$file")
            if [[ -f "${backup_dir}/${filename}" ]]; then
                cp "${backup_dir}/${filename}" "$file"
                echo "  ✅ 恢复: $file"
            fi
        done
        echo "🎉 备份恢复完成！"
    else
        echo "❌ 备份目录不存在: $backup_dir"
    fi
}

# 主函数
main() {
    cd "$SCRIPT_DIR"
    
    case "$1" in
        -l|--list)
            list_maps
            ;;
        -c|--current)
            show_current_map
            ;;
        -r|--restore)
            restore_backup
            ;;
        -h|--help|"")
            show_usage
            ;;
        *)
            local new_map=$1
            
            # 检查地图是否在可用列表中
            local found=0
            for map in "${AVAILABLE_MAPS[@]}"; do
                if [[ "$map" == "$new_map" ]]; then
                    found=1
                    break
                fi
            done
            
            if [[ $found -eq 0 ]]; then
                echo "❌ 错误: '$new_map' 不在可用地图列表中"
                echo
                echo "可用地图:"
                for map in "${AVAILABLE_MAPS[@]}"; do
                    echo "  - $map"
                done
                echo
                echo "要添加新地图，请编辑 map_config.sh 文件"
                exit 1
            fi
            
            if [[ "$new_map" == "$CURRENT_MAP" ]]; then
                echo "ℹ️  当前已经是地图: $new_map"
                show_current_map
            else
                switch_map "$new_map"
            fi
            ;;
    esac
}

# 运行主函数
main "$@"
