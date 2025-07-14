# 扫地机器人评估系统使用说明

## 概述

该评估系统基于图片中的指标体系设计，能够实时监控扫地机器人的性能指标，并在满足终止条件时自动保存评估结果为CSV格式。

## 支持的评估指标

根据图片中的指标体系，系统监控以下9个关键指标：

| 指标编号 | 指标名称 | 说明 | 计算方式 |
|---------|---------|------|---------|
| CR | 覆盖率 | 已清扫面积占总自由区域的比例 | 已覆盖面积 / 总自由面积 |
| ME | 运动效率 | 移动距离与清扫效果的比值 | 总移动距离 / 已覆盖面积 |
| SR | 清洁冗余度 | 重复清扫区域的比例 | 重复清扫面积 / 总清扫面积 |
| Collision | 碰撞次数 | 机器人碰撞或卡住的次数 | 通过位置停滞检测 |
| CT | 计算时间 | 路径规划的平均计算时间 | 目标规划的平均耗时 |
| FT | 任务总耗时 | 从开始到结束的总时间 | 系统运行总时长 |
| Vel_avg | 平均速度 | 机器人的平均移动速度 | 基于里程计数据计算 |
| Acc_avg | 平均加速度 | 机器人的平均加速度大小 | 速度变化率的均值 |
| Jerk_avg | 平均加加速度 | 加速度变化的平均值 | 加速度变化率的均值 |

## 终止条件

系统在以下两种情况下自动终止并保存结果：

1. **路径完成后覆盖率下降**：当所有规划路径点都已完成，且检测到覆盖率相比上一时刻有所下降时
2. **覆盖率停滞**：连续30秒内覆盖率增长不超过1%时

## 使用方法

### 1. 启动完整系统
```bash
cd /home/getting/Sweeping-Robot
source devel/setup.bash
roslaunch auto_nav sequential_clean.launch
```

### 2. 运行过程
- 系统会自动启动机器人仿真、地图服务、定位、路径规划等所有必要节点
- 评估器会实时监控机器人性能指标
- 在终端中会定期显示当前的覆盖率、路径长度和碰撞次数
- 满足终止条件时会自动保存结果并关闭节点

### 3. 查看结果
评估结果保存在 `/home/getting/Sweeping-Robot/evaluation_results/` 目录下，包含：

- `sweeping_metrics_steps_YYYYMMDD_HHMMSS.csv` - 详细的按步骤记录数据
- `sweeping_metrics_summary_YYYYMMDD_HHMMSS.csv` - 最终汇总指标
- `sweeping_evaluation_report_YYYYMMDD_HHMMSS.json` - 完整的评估报告

## 输出文件格式

### 汇总指标文件 (sweeping_metrics_summary_*.csv)
包含以下列：
- 实验时间
- 覆盖率_CR
- 运动效率_ME  
- 清洁冗余度_SR
- 碰撞次数_Collision
- 平均计算时间_CT
- 任务总耗时_FT
- 平均速度_Vel_avg
- 平均加速度_Acc_avg
- 平均加加速度_Jerk_avg
- 已覆盖面积_m2
- 总路径长度_m
- 完成目标数
- 总目标数

### 详细步骤文件 (sweeping_metrics_steps_*.csv)
包含每个时间步的所有指标值，用于分析性能变化趋势。

## 技术特点

1. **实时监控**：2Hz频率更新所有指标
2. **智能终止**：基于覆盖率变化自动判断任务完成
3. **全面指标**：涵盖效率、安全性、运动学等多个维度
4. **兼容性好**：自动检测pandas依赖，无pandas时使用内置csv模块
5. **数据完整**：同时保存CSV和JSON格式，便于后续分析

## 参数调整

可以通过修改脚本中的以下参数来调整评估行为：

```python
# 在 sweeping_robot_evaluator.py 中
self.robot_radius = 0.15  # 机器人清扫半径(米)
self.update_rate = 2.0    # 更新频率(Hz)
self.coverage_stagnation_threshold = 30.0  # 覆盖率停滞检测时间(秒)
self.coverage_growth_threshold = 0.01  # 覆盖率增长阈值(1%)
self.stuck_threshold = 0.05  # 碰撞检测位置阈值(米)
self.stuck_time_threshold = 3.0  # 碰撞检测时间阈值(秒)
```

## 故障排除

1. **权限问题**：确保脚本有执行权限
   ```bash
   chmod +x /home/getting/Sweeping-Robot/src/auto_nav/scripts/sweeping_robot_evaluator.py
   ```

2. **编译问题**：重新编译工作空间
   ```bash
   cd /home/getting/Sweeping-Robot && catkin_make
   ```

3. **依赖问题**：系统会自动适配，无需额外安装pandas

4. **输出目录问题**：系统会自动创建输出目录，确保有写入权限

## 注意事项

- 评估系统会替代原有的 `coverage_monitor.py` 节点
- 确保在仿真环境中运行，避免实际机器人安全问题
- 大型地图可能需要更长的运行时间才能满足终止条件
- 建议在稳定的网络环境中运行以避免ROS通信问题
