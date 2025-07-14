# 扫地机器人实时CSV数据保存功能说明

## 功能概述

在 `coverage_monitor.py v2.1` 中新增了每30秒自动保存评估数据到CSV文件的功能，可以实时记录机器人运行过程中的所有关键指标。

## 主要特性

### 1. 自动保存
- **间隔时间**: 每30秒自动保存一次
- **保存模式**: 追加模式，数据持续累积
- **文件格式**: CSV格式，便于Excel或Python分析

### 2. 数据内容
包含20个核心指标：

| 序号 | 字段名 | 说明 | 单位 |
|------|--------|------|------|
| 1 | Timestamp | 保存时间戳 | - |
| 2 | Runtime_s | 运行时间 | 秒 |
| 3 | Runtime_min | 运行时间 | 分钟 |
| 4 | Coverage_Rate | 覆盖率 | 0-1 |
| 5 | Motion_Efficiency | 运动效率 | m/m² |
| 6 | Redundancy | 冗余度 | 0-1 |
| 7 | Collision_Count | 碰撞次数 | 次 |
| 8 | Avg_Computation_Time | 平均计算时间 | 秒 |
| 9 | Total_Time | 总耗时 | 秒 |
| 10 | Avg_Velocity | 平均速度 | m/s |
| 11 | Avg_Acceleration | 平均加速度 | m/s² |
| 12 | Avg_Jerk | 平均加加速度 | m/s³ |
| 13 | Planned_Points | 规划路径点数 | 个 |
| 14 | Path_Length | 路径长度 | 米 |
| 15 | Covered_Area | 已覆盖面积 | m² |
| 16 | Redundant_Area | 重复面积 | m² |
| 17 | Free_Area_Total | 总自由面积 | m² |
| 18 | Path_Points_Count | 路径点数量 | 个 |
| 19 | Completed_Goals | 完成目标数 | 个 |
| 20 | Goal_Progress_Rate | 目标完成率 | % |

### 3. 文件管理
- **文件位置**: `/home/getting/tmp/sweeping_robot_realtime_data_<timestamp>.csv`
- **文件命名**: 包含启动时间戳，避免文件冲突
- **数据持久化**: 程序结束后CSV文件保留，可用于后续分析

## 使用方法

### 启动监控
```bash
# 1. 启动ROS核心
roscore

# 2. 在新终端启动覆盖监控
rosrun auto_nav coverage_monitor.py

# 3. 在新终端启动清扫任务
roslaunch auto_nav sequential_clean.launch map_name:=hospital_0.1
```

### 实时监控CSV文件生成
```bash
# 查看CSV文件
watch -n 5 'ls -la /home/getting/tmp/sweeping_robot_realtime_data_*.csv'

# 实时查看CSV内容
tail -f /home/getting/tmp/sweeping_robot_realtime_data_*.csv
```

## 数据分析示例

### Python分析脚本
```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV数据
df = pd.read_csv('/home/getting/tmp/sweeping_robot_realtime_data_<timestamp>.csv')

# 绘制覆盖率变化图
plt.figure(figsize=(12, 8))
plt.subplot(2, 2, 1)
plt.plot(df['Runtime_min'], df['Coverage_Rate'] * 100)
plt.title('覆盖率随时间变化')
plt.xlabel('运行时间(分钟)')
plt.ylabel('覆盖率(%)')

# 绘制运动效率图
plt.subplot(2, 2, 2)
plt.plot(df['Runtime_min'], df['Motion_Efficiency'])
plt.title('运动效率随时间变化')
plt.xlabel('运行时间(分钟)')
plt.ylabel('运动效率(m/m²)')

# 绘制速度图
plt.subplot(2, 2, 3)
plt.plot(df['Runtime_min'], df['Avg_Velocity'])
plt.title('平均速度随时间变化')
plt.xlabel('运行时间(分钟)')
plt.ylabel('平均速度(m/s)')

# 绘制碰撞累积图
plt.subplot(2, 2, 4)
plt.plot(df['Runtime_min'], df['Collision_Count'])
plt.title('碰撞次数累积')
plt.xlabel('运行时间(分钟)')
plt.ylabel('碰撞次数')

plt.tight_layout()
plt.show()
```

### Excel分析
1. 用Excel打开CSV文件
2. 选择数据创建图表
3. 分析关键指标趋势
4. 计算性能指标统计值

## 自动终止条件

系统支持以下自动终止条件（原有功能保持不变）：
- 覆盖率达到99%
- 30秒无运动进展
- 到达最后目标点

终止时会：
1. 保存最终文本报告
2. 保存最终CSV指标文件
3. 保留实时CSV数据文件供分析

## 日志输出

系统启动时会显示：
```
=== Coverage Monitor v2.1 Started ===
Robot cleaning radius: 0.15 meters
Update rate: 2.0 Hz
Auto CSV save interval: 30 seconds
Realtime CSV file: /home/getting/tmp/sweeping_robot_realtime_data_1673567890.csv
```

每次保存CSV时会输出：
```
已保存实时数据到CSV (运行时间: 60.0 s, 覆盖率: 15.67%)
```

## 注意事项

1. **文件大小**: 每30秒一行数据，长时间运行会积累较多数据
2. **存储位置**: 默认保存在/home/getting/tmp目录，重启后会清空
3. **性能影响**: CSV保存操作很轻量，不影响实时性能
4. **并发安全**: 支持多个任务同时运行，文件名不冲突

## 故障排除

### 常见问题
1. **CSV文件未生成**: 检查/home/getting/tmp目录权限
2. **数据不连续**: 检查ROS话题是否正常发布
3. **文件过大**: 可以定期清理/home/getting/tmp目录中的旧文件

### 调试命令
```bash
# 检查监控节点状态
rosnode info /coverage_monitor

# 查看话题数据
rostopic echo /coverage_percentage

# 检查CSV文件
ls -la /home/getting/tmp/sweeping_robot_realtime_data_*.csv
```

## 后续扩展

可以进一步扩展的功能：
1. 支持配置保存间隔
2. 添加数据压缩功能
3. 支持实时数据可视化
4. 集成数据库存储
5. 添加报警阈值监控
