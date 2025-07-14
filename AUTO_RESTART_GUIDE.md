# 自动重启功能使用指南

## 功能概述

扫地机器人覆盖率监控系统 v2.2 新增了智能自动重启功能，当检测到覆盖率停滞超过2分钟时，系统会自动重启整个清扫任务，确保机器人能够持续有效地进行清扫工作。

## 核心特性

### 1. 覆盖率停滞检测
- **检测阈值**: 2分钟 (120秒)
- **变化容差**: 0.1% (0.001)
- **检测频率**: 跟随系统更新频率 (2Hz)
- **检测原理**: 监控覆盖率变化，当连续2分钟变化小于0.1%时触发重启

### 2. 自动重启机制
- **触发条件**: 覆盖率停滞超过阈值
- **重启流程**: 
  1. 保存当前状态报告
  2. 优雅关闭所有ROS节点
  3. 清理进程资源
  4. 启动新的清扫任务
  5. 继续监控循环

### 3. 安全保护机制
- **最大重启次数**: 50次 (防止无限循环)
- **重启间隔**: 60秒 (确保系统稳定)
- **详细日志**: 记录每次重启的详细信息
- **状态报告**: 保存重启前的完整性能指标

## 使用方法

### 方式1: 自动重启管理器 (推荐)

```bash
# 启动自动重启管理器 (一键启动所有功能)
./auto_restart_manager.sh start

# 检查运行状态
./auto_restart_manager.sh status

# 查看运行日志
./auto_restart_manager.sh logs

# 停止管理器
./auto_restart_manager.sh stop
```

### 方式2: 手动启动 (调试用)

```bash
# 终端1: 启动roscore
roscore

# 终端2: 启动覆盖监控 (包含自动重启功能)
rosrun auto_nav coverage_monitor.py

# 终端3: 启动清扫任务
roslaunch auto_nav sequential_clean.launch map_name:=hospital_0.1
```

## 配置参数

### coverage_monitor.py 中的配置
```python
# 覆盖率停滞检测阈值 (秒)
self.coverage_stagnation_threshold = 120.0  # 2分钟

# 覆盖率变化最小容差
self.coverage_change_tolerance = 0.001  # 0.1%

# 是否启用自动重启
self.auto_restart_enabled = True

# 重启命令
self.restart_command = "roslaunch auto_nav sequential_clean.launch"
```

### auto_restart_manager.sh 中的配置
```bash
# 最大重启次数
MAX_RESTART_COUNT=10

# 重启间隔 (秒)
RESTART_DELAY=10

# 项目根目录
PROJECT_ROOT="/home/getting/Sweeping-Robot"

# 启动命令
LAUNCH_COMMAND="roslaunch auto_nav sequential_clean.launch"
```

## 日志和报告

### 1. 重启报告
- **位置**: `/tmp/auto_restart_report_<timestamp>.txt`
- **内容**: 重启前的完整状态，包括覆盖率、运行时间、性能指标等
- **格式**: 文本格式，便于阅读

### 2. 管理器日志
- **位置**: `/tmp/auto_restart_logs/restart_<N>_<timestamp>.log`
- **内容**: 每次重启的详细过程日志
- **管理**: 自动创建，按重启次数编号

### 3. CSV数据文件
- **位置**: `/tmp/sweeping_robot_realtime_data_<timestamp>.csv`
- **说明**: 每次重启会创建新的CSV文件，保持数据连续性

## 监控界面

### 实时状态监控
```bash
# 监控管理器状态
watch -n 5 './auto_restart_manager.sh status'

# 监控重启报告
watch -n 10 'ls -la /tmp/auto_restart_report_*.txt'

# 监控CSV文件生成
watch -n 5 'ls -la /tmp/sweeping_robot_realtime_data_*.csv'
```

### 日志实时查看
```bash
# 查看最新管理器日志
tail -f /tmp/auto_restart_logs/restart_*.log

# 查看覆盖监控输出
rostopic echo /coverage_percentage
```

## 使用场景

### 1. 机器人卡住场景
- **现象**: 机器人物理卡住，无法移动
- **检测**: 覆盖率停止增长
- **处理**: 2分钟后自动重启，机器人重新开始清扫

### 2. 路径规划失败场景
- **现象**: 路径规划算法失效，机器人停止移动
- **检测**: 覆盖率长时间无变化
- **处理**: 自动重启重新规划路径

### 3. 定位漂移场景
- **现象**: AMCL定位严重漂移，导致导航失效
- **检测**: 覆盖率计算异常，停止增长
- **处理**: 重启系统重新初始化定位

### 4. 传感器故障场景
- **现象**: 激光雷达或其他传感器临时故障
- **检测**: 机器人停止有效移动
- **处理**: 重启系统重新初始化传感器

## 性能影响

### 1. 系统开销
- **CPU使用**: 覆盖率检测开销极小 (<1%)
- **内存使用**: 状态记录占用内存很少 (<10MB)
- **网络影响**: 无额外网络开销

### 2. 重启时间
- **检测延迟**: 2分钟 (可配置)
- **关闭时间**: 3-5秒
- **启动时间**: 10-15秒
- **总重启时间**: 约20秒

## 故障排除

### 1. 自动重启不工作
```bash
# 检查coverage_monitor是否运行
rosnode list | grep coverage_monitor

# 检查覆盖率话题
rostopic echo /coverage_percentage

# 查看监控日志
rosnode logs coverage_monitor
```

### 2. 重启次数过多
```bash
# 检查重启报告找出原因
cat /tmp/auto_restart_report_*.txt

# 调整参数减少误触发
# 增加coverage_stagnation_threshold或减少coverage_change_tolerance
```

### 3. 管理器无响应
```bash
# 检查管理器进程
ps aux | grep auto_restart_manager

# 手动停止并重启
./auto_restart_manager.sh stop
./auto_restart_manager.sh start
```

## 最佳实践

### 1. 参数调优
- **测试环境**: 先在仿真环境中测试合适的阈值
- **环境适配**: 根据实际环境调整停滞检测时间
- **性能平衡**: 在响应速度和稳定性间找到平衡

### 2. 监控建议
- **定期检查**: 定期查看重启报告分析原因
- **日志分析**: 通过日志优化系统参数
- **性能监控**: 监控重启频率是否合理

### 3. 维护建议
- **日志清理**: 定期清理临时文件和日志
- **参数更新**: 根据使用经验调整配置参数
- **系统优化**: 根据重启原因优化底层系统

## 高级配置

### 1. 自定义重启条件
```python
# 在coverage_monitor.py中添加额外条件
def check_additional_restart_conditions(self):
    # 例如：检查机器人速度
    if self.avg_velocity < 0.01 and self.running_time > 300:
        return True
    # 例如：检查路径点密度
    if len(self.path_history) < 10 and self.running_time > 600:
        return True
    return False
```

### 2. 自定义重启命令
```bash
# 修改auto_restart_manager.sh中的启动命令
LAUNCH_COMMAND="roslaunch auto_nav sequential_clean.launch map_name:=custom_map"
```

### 3. 集成外部监控
```python
# 发布重启信号到自定义话题
self.restart_signal_pub = rospy.Publisher('/system/restart_signal', String, queue_size=1)
```

这个自动重启功能为扫地机器人系统提供了强大的故障恢复能力，确保系统能够在遇到各种问题时自动恢复并继续工作，大大提高了系统的可靠性和实用性。
