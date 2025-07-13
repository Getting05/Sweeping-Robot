# 清扫机器人顺序路径执行说明

## 项目逻辑修改
已将项目逻辑修改为：生成路径点后，机器人按着路径点一个一个前进。

## 核心文件
1. **sequential_goal.cpp** - 新的路径执行逻辑，按顺序访问每个路径点
2. **coverage_monitor.py** - 覆盖率监控脚本，实时显示清扫进度

## 启动文件选项

### 1. 带图形界面（调试用）
```bash
source devel/setup.bash
roslaunch auto_nav sequential_clean.launch
```

### 2. 无图形界面（加快运行速度）
```bash
source devel/setup.bash
roslaunch auto_nav sequential_clean_headless.launch
```

### 3. 带覆盖率监控（推荐）
```bash
source devel/setup.bash
roslaunch auto_nav sequential_clean_with_monitor.launch
```

## 新逻辑特点

### sequential_goal节点
- ✅ 按顺序访问路径点，不会跳跃
- ✅ 智能超时处理（45秒）
- ✅ 连续超时保护机制
- ✅ 实时距离计算和调试信息
- ✅ 自动任务完成检测

### 覆盖率监控
- ✅ 实时覆盖率计算
- ✅ 路径长度统计
- ✅ 目标完成率追踪
- ✅ 每5秒更新一次报告
- ✅ 自动保存最终报告到文件

## 运行参数
- 目标容差：0.3米
- 目标超时：45秒
- 清扫半径：0.15米
- 机器人半径：0.12米

## 输出文件
运行完成后会在项目根目录生成 `coverage_report.txt` 文件，包含：
- 运行时间
- 总路径长度
- 区域覆盖率
- 目标完成率
- 平均速度
- 清扫效率

## 使用建议
1. 首次测试建议使用带图形界面的版本
2. 正式运行建议使用无图形界面版本以加快速度
3. 需要详细统计时使用带覆盖率监控的版本
