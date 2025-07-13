#!/bin/bash

# 清扫机器人覆盖率监控系统演示脚本

echo "🤖 清扫机器人覆盖率监控系统演示"
echo "========================================"

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}📋 项目完成状态检查...${NC}"
echo ""

# 检查核心文件
echo "✅ 核心文件检查:"
files=(
    "src/auto_nav/src/sequential_goal.cpp"
    "src/auto_nav/scripts/coverage_monitor.py"
    "src/auto_nav/scripts/system_test.py"
    "src/auto_nav/launch/sequential_clean_with_coverage.launch"
    "start_coverage_system.sh"
    "COVERAGE_SYSTEM_GUIDE.md"
    "QUICK_START.md"
    "PROJECT_STATUS.md"
)

for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo "   ✓ $file"
    else
        echo "   ✗ $file (缺失)"
    fi
done

echo ""
echo -e "${BLUE}🔧 系统功能演示...${NC}"
echo ""

# 检查可执行权限
echo "✅ 可执行权限检查:"
scripts=(
    "src/auto_nav/scripts/coverage_monitor.py"
    "src/auto_nav/scripts/system_test.py"
    "src/auto_nav/scripts/test_coverage_monitor.py"
    "start_coverage_system.sh"
)

for script in "${scripts[@]}"; do
    if [ -x "$script" ]; then
        echo "   ✓ $script"
    else
        echo "   ✗ $script (无执行权限)"
    fi
done

echo ""
echo -e "${BLUE}📊 编译状态检查...${NC}"

if [ -f "devel/lib/auto_nav/sequential_goal" ]; then
    echo "   ✓ sequential_goal 已编译"
else
    echo "   ✗ sequential_goal 未编译"
fi

echo ""
echo -e "${BLUE}🚀 快速启动选项:${NC}"
echo ""
echo "1. 仿真模式:"
echo "   ./start_coverage_system.sh sim"
echo ""
echo "2. 真实机器人模式:"
echo "   ./start_coverage_system.sh real"
echo ""
echo "3. 系统测试:"
echo "   ./start_coverage_system.sh test"
echo ""
echo "4. 手动启动覆盖率监控:"
echo "   roslaunch auto_nav sequential_clean_with_coverage.launch"
echo ""

echo -e "${GREEN}🎉 项目开发完成！${NC}"
echo ""
echo "📖 详细使用说明请参考："
echo "   - QUICK_START.md (快速开始)"
echo "   - COVERAGE_SYSTEM_GUIDE.md (完整指南)"
echo "   - PROJECT_STATUS.md (项目状态)"
echo ""

echo -e "${YELLOW}💡 建议下一步：${NC}"
echo "   运行 './start_coverage_system.sh test' 进行系统测试"
echo ""
