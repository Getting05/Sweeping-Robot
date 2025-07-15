#!/bin/bash

# 快速测试算法选择功能的脚本

echo "=== 测试扫地机器人算法选择功能 ==="

# 模拟用户输入测试
test_smart_start() {
    echo "测试智能启动功能..."
    
    # 检查脚本是否存在
    if [ ! -f "/home/getting/Sweeping-Robot/start_intelligent_cleaning.sh" ]; then
        echo "❌ 启动脚本不存在"
        return 1
    fi
    
    # 检查函数定义
    if grep -q "smart_start()" "/home/getting/Sweeping-Robot/start_intelligent_cleaning.sh"; then
        echo "✅ smart_start函数存在"
    else
        echo "❌ smart_start函数不存在"
        return 1
    fi
    
    # 检查算法选择代码
    if grep -q "路径规划算法选择" "/home/getting/Sweeping-Robot/start_intelligent_cleaning.sh"; then
        echo "✅ 算法选择菜单已添加"
    else
        echo "❌ 算法选择菜单未找到"
        return 1
    fi
    
    # 检查参数配置代码
    if grep -q "算法参数配置" "/home/getting/Sweeping-Robot/start_intelligent_cleaning.sh"; then
        echo "✅ 参数配置功能已添加"
    else
        echo "❌ 参数配置功能未找到"
        return 1
    fi
    
    # 检查临时配置文件生成
    if grep -q "temp_config=" "/home/getting/Sweeping-Robot/start_intelligent_cleaning.sh"; then
        echo "✅ 临时配置文件生成功能已添加"
    else
        echo "❌ 临时配置文件生成功能未找到"
        return 1
    fi
    
    echo "✅ 所有功能检查通过！"
    return 0
}

# 显示使用方法
show_usage() {
    echo ""
    echo "=== 新功能使用方法 ==="
    echo ""
    echo "1. 启动智能清扫系统："
    echo "   ./start_intelligent_cleaning.sh smart"
    echo ""
    echo "2. 系统会提示您选择："
    echo "   - 地图名称"
    echo "   - 路径规划算法 (A*或神经网络)"
    echo "   - 算法参数设置"
    echo ""
    echo "3. 支持的算法："
    echo "   - A*算法: 高质量路径规划，适合精确覆盖"
    echo "   - 神经网络: 快速响应，适合实时应用"
    echo ""
    echo "4. 可配置参数："
    echo "   A*算法:"
    echo "   - 启发式权重 (影响路径搜索方向)"
    echo "   - 覆盖策略 (nearest/farthest/spiral)"
    echo "   - 最大迭代次数"
    echo ""
    echo "   神经网络算法:"
    echo "   - 网络参数 c_0"
    echo "   - 最大迭代次数"
    echo ""
}

# 运行测试
echo "开始功能测试..."
if test_smart_start; then
    echo ""
    echo "🎉 测试完成！算法选择功能已成功集成到智能启动脚本中。"
    show_usage
else
    echo ""
    echo "❌ 测试失败，请检查脚本修改。"
    exit 1
fi
