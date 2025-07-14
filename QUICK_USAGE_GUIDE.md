# 扫地机器人评估系统快速使用指南

## 启动命令
```bash
cd /home/getting/Sweeping-Robot
source devel/setup.bash
roslaunch auto_nav sequential_clean.launch
```

## 系统功能

### 1. 实时监控覆盖率（保留原功能）
在另一个终端运行以下命令可以实时查看覆盖率：
```bash
# 新终端
cd /home/getting/Sweeping-Robot
source devel/setup.bash
rostopic echo /coverage_percentage
```

### 2. 自动保存CSV指标（新功能）
系统会自动监控以下9个指标：
- **CR**: 覆盖率
- **ME**: 运动效率
- **SR**: 清洁冗余度  
- **Collision**: 碰撞次数
- **CT**: 计算时间
- **FT**: 任务总耗时
- **Vel_avg**: 平均速度
- **Acc_avg**: 平均加速度
- **Jerk_avg**: 平均加加速度

## 自动终止条件
系统在以下情况下自动保存结果并结束：
1. 到达最后一个路径点后，下一刻覆盖率下降
2. 30秒内覆盖率增长不超过1%

## 输出文件位置
```
/home/getting/Sweeping-Robot/evaluation_results/
├── sweeping_metrics_steps_YYYYMMDD_HHMMSS.csv     # 详细步骤数据
├── sweeping_metrics_summary_YYYYMMDD_HHMMSS.csv   # 最终汇总指标
└── sweeping_evaluation_report_YYYYMMDD_HHMMSS.json # 完整评估报告
```

## 实时监控其他信息
```bash
# 查看机器人当前位置
rostopic echo /odom

# 查看规划路径
rostopic echo /plan_path

# 查看已走过路径
rostopic echo /passedPath

# 查看当前目标
rostopic echo /move_base_simple/goal
```

## 注意事项
- 评估系统会在终端显示当前状态信息
- 两个监控系统同时运行，不会互相干扰
- 原有的实时覆盖率查看功能完全保留
- 新的评估系统会自动检测任务完成并保存所有指标
