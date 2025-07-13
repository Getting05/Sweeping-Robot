#!/bin/bash

echo "🗺️  地图管理系统使用指南"
echo "========================="
echo
echo "📋 功能概述:"
echo "这个地图管理系统可以让您轻松地在不同地图之间切换，"
echo "无需手动编辑多个launch文件。"
echo
echo "🚀 快速使用:"
echo
echo "1. 查看当前地图:"
echo "   ./switch_map.sh --current"
echo
echo "2. 列出所有可用地图:"
echo "   ./switch_map.sh --list"
echo
echo "3. 切换到hospital_0.1地图:"
echo "   ./switch_map.sh hospital_0.1"
echo
echo "4. 切换到kitchen_5地图:"
echo "   ./switch_map.sh kitchen_5"
echo
echo "5. 启动图形化管理界面:"
echo "   ./map_manager.sh"
echo
echo "📁 文件说明:"
echo "- map_config.sh        # 地图配置文件"
echo "- switch_map.sh        # 地图切换脚本"
echo "- map_manager.sh       # 图形化管理界面"
echo "- parameterize_launch.sh # launch文件参数化工具"
echo
echo "🔧 添加新地图:"
echo "1. 将.yaml和.pgm文件放入 src/auto_nav/map/ 目录"
echo "2. 运行: ./map_manager.sh"
echo "3. 选择 '4. 添加新地图到配置'"
echo "4. 输入地图名称（不含扩展名）"
echo
echo "💾 备份机制:"
echo "每次切换地图时，系统会自动备份所有launch文件"
echo "备份文件夹格式: launch_backup_YYYYMMDD_HHMMSS"
echo
echo "⚡ 批量操作示例:"
echo
echo "# 切换到hospital地图并启动系统"
echo "./switch_map.sh hospital_0.1"
echo "catkin_make"
echo "./start_coverage_system.sh"
echo
echo "# 切换回kitchen地图"
echo "./switch_map.sh kitchen_5"
echo
echo "🎯 支持的地图:"
echo "- kitchen_5 (厨房地图)"
echo "- hospital_0.1 (医院地图)"
echo "- 其他自定义地图 (可通过配置添加)"
echo
echo "📞 如需帮助:"
echo "./switch_map.sh --help"
echo "./map_manager.sh"
echo
echo "✨ 享受便捷的地图管理体验！"
