# 清扫机器人覆盖率监控系统使用指南

## 系统概述

本系统实现了基于ROS的移动扫地机器人实时覆盖率监控功能，包括：

- **实时覆盖率计算**：计算机器人已清扫面积占自由区域面积的百分比
- **历史轨迹保持**：红线（已清扫路径）完整保留历史轨迹，不被刷新
- **可视化支持**：在RViz中实时显示覆盖率和清扫进度
- **鲁棒性优化**：解决路径漂移、瞬移、红线丢失等问题

## 快速启动

### 1. 环境准备

确保已安装ROS和相关依赖：
```bash
# 确保ROS环境已source
source /opt/ros/noetic/setup.bash  # 根据你的ROS版本调整

# 进入工作空间
cd /home/getting/Sweeping-Robot

# source工作空间
source devel/setup.bash
```

### 2. 一键启动

使用提供的启动脚本：

```bash
# 仿真模式（包含Gazebo）
./start_coverage_system.sh sim

# 真实机器人模式
./start_coverage_system.sh real

# 仅可视化模式
./start_coverage_system.sh viz

# 系统测试模式
./start_coverage_system.sh test
```

### 3. 手动启动（可选）

如果需要更细粒度的控制：

```bash
# 启动仿真+导航+覆盖率监控
roslaunch auto_nav sequential_clean_with_coverage.launch

# 启动可视化+覆盖率监控（适用于真实机器人）
roslaunch auto_nav sequential_clean_with_coverage_viz.launch
```

## 主要功能

### 1. 覆盖率监控

- **话题**: `/coverage_percentage`
- **类型**: `std_msgs/Float32`
- **功能**: 实时发布覆盖率百分比

```bash
# 监控覆盖率
rostopic echo /coverage_percentage
```

### 2. 已清扫路径（红线）

- **话题**: `/cleaned_path`
- **类型**: `nav_msgs/Path`
- **功能**: 发布机器人历史清扫轨迹，红线在RViz中永久显示

### 3. 覆盖区域可视化

- **话题**: `/covered_area_grid`
- **类型**: `nav_msgs/OccupancyGrid`
- **功能**: 发布已覆盖区域的栅格地图

### 4. 实时统计信息

覆盖率监控节点会在终端输出详细统计：
- 总路径长度
- 已覆盖面积
- 自由区域面积
- 覆盖率百分比
- 清扫效率

## 系统架构

### 核心节点

1. **sequential_goal** - 自动清扫导航节点
   - 路径: `src/auto_nav/src/sequential_goal.cpp`
   - 功能: 自动导航和路径规划，发布已清扫路径

2. **coverage_monitor** - 覆盖率监控节点
   - 路径: `src/auto_nav/scripts/coverage_monitor.py`
   - 功能: 计算覆盖率，发布统计信息

### 关键特性

#### 1. 坐标系统一
- 所有导航和可视化均使用`map`坐标系
- `odom`位置实时转换到`map`坐标系
- 避免坐标系混用导致的漂移问题

#### 2. 历史轨迹保持
- 红线路径采用永久累积策略
- 只在首次接收路径时清空历史
- 后续路径更新保留所有历史点
- 避免路径重置导致的轨迹丢失

#### 3. 覆盖率算法
- 基于路径点的覆盖区域计算
- 支持自定义覆盖半径
- 实时更新，避免重复计算
- 高效的栅格地图处理

## 配置参数

### 覆盖率监控参数

在`coverage_monitor.py`中可调整：

```python
# 覆盖半径（米）
COVERAGE_RADIUS = 0.3

# 路径点最小间距（米）
MIN_POINT_DISTANCE = 0.1

# 发布频率（Hz）
PUBLISH_RATE = 2.0

# 最大历史路径点数
MAX_HISTORY_POINTS = 10000
```

### 导航参数优化

关键配置文件：
- `src/auto_nav/config/base_local_planner_params.yaml` - 局部规划器参数
- `src/auto_nav/launch/amcl.launch` - 定位参数

## 测试与验证

### 1. 系统完整性测试

```bash
# 运行自动化测试（30秒）
./start_coverage_system.sh test

# 或手动运行
python3 src/auto_nav/scripts/system_test.py 30
```

### 2. 覆盖率监控测试

```bash
# 监控覆盖率话题
python3 src/auto_nav/scripts/test_coverage_monitor.py
```

### 3. 手动验证

1. **启动系统**后，在RViz中应该看到：
   - 机器人模型
   - 地图
   - 绿色路径（当前规划路径）
   - 红色路径（已清扫轨迹）
   - 覆盖区域栅格地图

2. **话题检查**：
   ```bash
   # 检查所有相关话题
   rostopic list | grep -E "(coverage|cleaned_path|covered_area)"
   
   # 检查话题频率
   rostopic hz /coverage_percentage
   rostopic hz /cleaned_path
   ```

## 故障排除

### 常见问题

1. **红线不显示或消失**
   - 检查`/cleaned_path`话题是否正常发布
   - 确认RViz中Path显示设置正确
   - 重启sequential_goal节点

2. **覆盖率为0或不更新**
   - 检查地图是否正确加载
   - 确认coverage_monitor.py正在运行
   - 检查机器人位置是否正常

3. **路径漂移或跳跃**
   - 检查TF树是否正常
   - 调整AMCL参数
   - 确认传感器数据质量

### 日志检查

```bash
# 查看覆盖率监控日志
rosnode info /coverage_monitor

# 查看导航节点日志
rosnode info /sequential_goal

# 查看所有活跃节点
rosnode list
```

## 性能优化建议

1. **大规模环境**：
   - 增加`MAX_HISTORY_POINTS`限制
   - 调整`MIN_POINT_DISTANCE`避免过密的路径点
   - 降低发布频率减少计算负载

2. **精度要求高**：
   - 减小`COVERAGE_RADIUS`提高精度
   - 减小`MIN_POINT_DISTANCE`增加路径密度
   - 提高传感器更新频率

3. **实时性要求高**：
   - 增加发布频率
   - 优化路径点处理算法
   - 使用更高性能的硬件

## 扩展功能

系统设计为可扩展架构，可以轻松添加：

- 清扫质量评估
- 多机器人协同清扫
- 动态环境适应
- 清扫任务调度
- 远程监控界面

## 技术支持

如遇到问题，请检查：
1. ROS环境是否正确配置
2. 所有依赖包是否安装
3. 传感器数据是否正常
4. 网络连接是否稳定

详细的错误信息可通过ROS日志查看：
```bash
rqt_console  # 图形化日志查看器
```
