#!/bin/bash

# 创建参数化launch文件的脚本
# 将硬编码的地图名称改为参数形式

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 要参数化的launch文件
LAUNCH_FILES=(
    "src/auto_nav/launch/clean_work_sequential.launch"
    "src/auto_nav/launch/sequential_clean.launch"
    "src/auto_nav/launch/sequential_clean_with_coverage.launch"
    "src/auto_nav/launch/sequential_clean_with_monitor.launch"
    "src/auto_nav/launch/clean_work_headless.launch"
    "src/auto_nav/launch/clean_work.launch"
    "src/auto_nav/launch/sequential_clean_with_coverage_viz.launch"
    "src/auto_nav/launch/sequential_clean_headless.launch"
)

# 创建参数化版本
create_parameterized_launch() {
    echo "🔧 创建参数化的launch文件..."
    
    for file in "${LAUNCH_FILES[@]}"; do
        if [[ -f "$file" ]]; then
            echo "  处理: $file"
            
            # 备份原文件
            cp "$file" "${file}.backup"
            
            # 在launch文件开头添加参数定义
            local temp_file=$(mktemp)
            
            cat > "$temp_file" << 'EOF'
<launch>
  <!-- 地图参数 - 可以通过命令行参数或其他launch文件传递 -->
  <arg name="map_name" default="kitchen_5" />
  <arg name="map_file" default="$(find auto_nav)/map/$(arg map_name).yaml" />
  
EOF
            
            # 添加原文件内容，但跳过第一行的<launch>
            tail -n +2 "$file" >> "$temp_file"
            
            # 替换地图路径
            sed -i 's|$(find auto_nav)/map/[^"]*\.yaml|$(arg map_file)|g' "$temp_file"
            
            # 替换原文件
            mv "$temp_file" "$file"
            
            echo "    ✅ 已参数化"
        else
            echo "    ⚠️  文件不存在: $file"
        fi
    done
    
    echo "🎉 参数化完成！"
    echo
    echo "💡 现在可以这样启动launch文件:"
    echo "  roslaunch auto_nav clean_work_sequential.launch map_name:=hospital_0.1"
    echo "  roslaunch auto_nav clean_work_sequential.launch map_name:=kitchen_5"
}

# 恢复原始版本
restore_original_launch() {
    echo "🔄 恢复原始launch文件..."
    
    for file in "${LAUNCH_FILES[@]}"; do
        local backup_file="${file}.backup"
        if [[ -f "$backup_file" ]]; then
            cp "$backup_file" "$file"
            echo "  ✅ 恢复: $file"
        else
            echo "  ⚠️  备份不存在: $backup_file"
        fi
    done
    
    echo "🎉 恢复完成！"
}

# 显示使用说明
show_usage() {
    echo "Launch文件参数化工具"
    echo "用法: $0 [选项]"
    echo
    echo "选项:"
    echo "  -p, --parameterize    创建参数化的launch文件"
    echo "  -r, --restore         恢复原始launch文件"
    echo "  -h, --help            显示此帮助信息"
    echo
    echo "参数化后的使用方法:"
    echo "  roslaunch auto_nav clean_work_sequential.launch map_name:=hospital_0.1"
}

# 主函数
main() {
    cd "$SCRIPT_DIR"
    
    case "$1" in
        -p|--parameterize)
            create_parameterized_launch
            ;;
        -r|--restore)
            restore_original_launch
            ;;
        -h|--help|"")
            show_usage
            ;;
        *)
            echo "❌ 未知选项: $1"
            show_usage
            exit 1
            ;;
    esac
}

main "$@"
