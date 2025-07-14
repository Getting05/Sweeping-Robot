# 覆盖率监控调试指南

## 问题诊断

如果覆盖率显示过低（如0.02%），可能的原因和解决方案：

### 1. 机器人清扫半径问题
**问题**: 机器人清扫半径设置过小
**解决**: 已将半径从0.15m增加到0.3m

### 2. 路径数据问题
**检查方法**:
```bash
# 检查是否有路径数据
rostopic echo /passedPath

# 检查路径点数量
rostopic echo /passedPath | grep -c "position"
```

### 3. 坐标系转换问题
**常见问题**: odom坐标系与map坐标系不一致
**检查方法**:
```bash
# 检查TF变换
rosrun tf tf_echo map odom

# 检查机器人当前位置
rostopic echo /odom
```

### 4. 地图加载问题
**检查方法**:
```bash
# 检查地图话题
rostopic echo /map | head -20

# 检查地图服务器
rosnode info /map_server
```

## 参数调整

在 `coverage_monitor.py` 中可以调整的参数：

```python
# 机器人清扫半径 - 增加可提高覆盖率
self.robot_radius = 0.3  # 米

# 最小点间距 - 减少可增加采样密度
self.min_point_distance = 0.05  # 米

# 更新频率 - 增加可提高响应性
self.update_rate = 2.0  # Hz
```

## 调试步骤

1. **启动系统**:
   ```bash
   roslaunch auto_nav sequential_clean.launch
   ```

2. **监控覆盖率**:
   ```bash
   rostopic echo /coverage_percentage
   ```

3. **检查详细日志**:
   观察终端输出的统计信息，关注：
   - 已覆盖栅格数
   - 历史路径点数
   - 地图分辨率和尺寸

4. **可视化检查**:
   在RViz中添加话题：
   - `/covered_area_grid` - 查看已覆盖区域
   - `/passedPath` - 查看机器人路径

## 预期值

正常情况下：
- 覆盖率应该逐渐增长，从0%开始
- 小房间（如kitchen_5）预期最终覆盖率：60-80%
- 大房间预期最终覆盖率：40-60%

## 故障排除

**如果覆盖率始终为0**:
1. 检查机器人是否真的在移动
2. 检查/passedPath话题是否有数据
3. 检查地图是否正确加载

**如果覆盖率增长太慢**:
1. 增加机器人清扫半径
2. 减少最小点间距
3. 检查机器人移动速度
