# 清扫机器人覆盖率监控系统 - 快速使用指南

## 🎯 核心功能
- ✅ **实时覆盖率监控**：显示清扫覆盖百分比
- ✅ **历史轨迹保持**：红线路径永久保留，不被重置
- ✅ **可视化支持**：RViz中实时显示覆盖进度

## 🚀 快速启动

### 1. 仿真模式（推荐首次使用）
```bash
cd /home/getting/Sweeping-Robot
./start_coverage_system.sh sim
```

### 2. 真实机器人模式
```bash
cd /home/getting/Sweeping-Robot
./start_coverage_system.sh real
```

### 3. 系统测试
```bash
cd /home/getting/Sweeping-Robot
./start_coverage_system.sh test
```

## 📊 监控覆盖率
```bash
# 实时查看覆盖率百分比
rostopic echo /coverage_percentage

# 监控详细统计信息
python3 src/auto_nav/scripts/test_coverage_monitor.py
```

## 🔧 RViz可视化设置
启动后在RViz中添加以下显示项：
1. **Path** - 订阅 `/cleaned_path` (红色，已清扫路径)
2. **Path** - 订阅 `/move_base/NavfnROS/plan` (绿色，当前规划)
3. **Map** - 订阅 `/covered_area_grid` (已覆盖区域)

## 📖 详细文档
- 完整使用指南：[COVERAGE_SYSTEM_GUIDE.md](COVERAGE_SYSTEM_GUIDE.md)
- 项目状态报告：[PROJECT_STATUS.md](PROJECT_STATUS.md)

## ⚡ 快速故障排除
- **红线不显示**：检查RViz中Path设置，确认话题为`/cleaned_path`
- **覆盖率为0**：确认机器人正在移动，地图已正确加载
- **启动失败**：检查ROS环境，运行`source devel/setup.bash`

---
🏆 **系统状态：生产就绪，可直接使用**
